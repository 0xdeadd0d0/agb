use core::marker::PhantomData;
use core::ptr::addr_of;
use core::ptr::addr_of_mut;

/// 4000134h - RCNT (R) - Mode Selection, in Normal/Multiplayer/UART modes (R/W)
const SERIAL_RCNT: *mut u16 = (0x04000134) as *mut u16;
const SERIAL_BASE_ADDR: usize = 0x04000120;

/// Serial multi-player flags -------------------------------------------
const SERIAL_RCNT_BIT_SLAVE: u16 = 2;
const SERIAL_RCNT_BIT_BIT_READY: u16 = 3;
const SERIAL_RCNT_BITS_PLAYER_ID: u16 = 4;
const SERIAL_RCNT_BIT_ERROR: u16 = 6;
const SERIAL_RCNT_BIT_START: u16 = 7;
const SERIAL_RCNT_BIT_MULTIPLAYER: u16 = 13;
const SERIAL_RCNT_BIT_IRQ: u16 = 14;
const SERIAL_RCNT_BIT_GENERAL_PURPOSE_LOW: u16 = 14;
const SERIAL_RCNT_BIT_GENERAL_PURPOSE_HIGH: u16 = 15;

#[repr(u16)]
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Clone, Copy)]
pub enum SerialBaudRate {
    BaudRate0 = 0, // 9600 bps
    BaudRate1 = 1, // 38400 bps
    BaudRate2 = 2, // 57600 bps
    BaudRate3 = 3, // 115200 bps
}

impl SerialBaudRate {
    fn discriminant(&self) -> u16 {
        unsafe { *(self as *const Self as *const u16) }
    }
}

#[allow(dead_code)]
#[derive(Clone, Copy)]
#[repr(C, packed)]
struct SerialMultiPlayerReg {
    sio_multi_data_0: u16,
    sio_multi_data_1: u16,
    sio_multi_data_2: u16,
    sio_multi_data_3: u16,
    sio_cnt: u16,
    sio_multi_data_send: u16,
}

#[allow(dead_code)]
#[derive(Clone, Copy)]
#[repr(C, packed)]
struct SerialNormal {
    sio_data32_lsb: u16,
    sio_data32_msb: u16,
    sio_rfu0: u16,
    sio_rfu1: u16,
    sio_cnt: u16,
    sio_data8: u8,
    sio_rfu2: [u8; 3],
}

#[allow(dead_code)]
#[derive(Clone, Copy)]
#[repr(C, packed)]
struct SerialUart {
    sio_rfu0: u16,
    sio_rfu1: u16,
    sio_rfu2: u16,
    sio_rfu3: u16,
    sio_cnt_l: u16,
    sio_data8: u8,
    sio_rfu4: [u8; 3],
}

#[allow(dead_code)]
#[derive(Clone, Copy)]
#[repr(C, packed)]
struct SerialGpio {
    sio_rfu0: u16,
    sio_rfu1: u16,
    sio_rfu2: u16,
    sio_rfu3: u16,
    sio_rfu4: u16,
    sio_rfu5: u16,
}

#[repr(C)]
union Serial {
    multiplay_mode_reg: SerialMultiPlayerReg,
    normal_mode_reg: SerialNormal,
    uart_mode_reg: SerialUart,
    gpio_mode_reg: SerialGpio,
}

struct SerialResponse {
    sio_data: [u16; 4],
    sio_player_id: u32,
}

const SIO_INVALID_PLAYER_ID: u32 = 0xDEADBEEF;

impl Serial {
    fn new() -> &'static mut Serial {
        unsafe { &mut *(SERIAL_BASE_ADDR as *mut Self) }
    }

    fn set_multi_player_mode(&mut self, baud_rate: SerialBaudRate) {
        unsafe {
            SERIAL_RCNT.write_volatile(0);
            *addr_of_mut!(self.multiplay_mode_reg.sio_cnt) =
                (1 << SERIAL_RCNT_BIT_MULTIPLAYER) | (baud_rate.discriminant());
            *addr_of_mut!(self.multiplay_mode_reg.sio_multi_data_send) = 0;
        }
    }

    fn set_general_purpose_mode(&mut self) {
        unsafe {
            *addr_of_mut!(self.multiplay_mode_reg.sio_cnt) = (1
                << SERIAL_RCNT_BIT_GENERAL_PURPOSE_LOW)
                | (1 << SERIAL_RCNT_BIT_GENERAL_PURPOSE_HIGH);
            *addr_of_mut!(self.multiplay_mode_reg.sio_multi_data_send) = 0;
        }
    }

    fn set_data(&mut self, data: u16) {
        unsafe {
            *addr_of_mut!(self.multiplay_mode_reg.sio_multi_data_send) = data;
        }
    }

    fn get_data<'a>(&self, rsp: &'a mut SerialResponse) -> &'a SerialResponse {
        unsafe {
            rsp.sio_player_id = SIO_INVALID_PLAYER_ID;
            rsp.sio_data[0] = *addr_of!(self.multiplay_mode_reg.sio_multi_data_0);
            rsp.sio_data[1] = *addr_of!(self.multiplay_mode_reg.sio_multi_data_1);
            rsp.sio_data[2] = *addr_of!(self.multiplay_mode_reg.sio_multi_data_2);
            rsp.sio_data[3] = *addr_of!(self.multiplay_mode_reg.sio_multi_data_3);
            rsp.sio_player_id = *addr_of!(self.multiplay_mode_reg.sio_cnt) as u32
                & SERIAL_RCNT_BITS_PLAYER_ID as u32 >> SERIAL_RCNT_BITS_PLAYER_ID as u32;
        }
        rsp
    }
    #[inline(always)]
    fn start_transfer(&mut self) {
        unsafe {
            *addr_of_mut!(self.multiplay_mode_reg.sio_cnt) |= 1 << SERIAL_RCNT_BIT_START;
        }
    }
}

impl core::fmt::Display for Serial {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        unsafe {
            let rcnt_str = SERIAL_RCNT.read_volatile();
            let sio_multi_data_0_str = *addr_of!(self.multiplay_mode_reg.sio_multi_data_0);
            let sio_multi_data_1_str = *addr_of!(self.multiplay_mode_reg.sio_multi_data_1);
            let sio_multi_data_2_str = *addr_of!(self.multiplay_mode_reg.sio_multi_data_2);
            let sio_multi_data_3_str = *addr_of!(self.multiplay_mode_reg.sio_multi_data_3);
            let sio_cnt_str = *addr_of!(self.multiplay_mode_reg.sio_cnt);
            let sio_multi_data_send_str = *addr_of!(self.multiplay_mode_reg.sio_multi_data_send);
            write!(
                f,
                "SERIAL_RCNT: {:x}\nSERIAL_SIOMULTI0: {:x}\nSERIAL_SIOMULTI1: {:x}\n\
        SERIAL_SIOMULTI2: {:x}\nSERIAL_SIOMULTI3: {:x}\nSERIAL_SIOCNT: {:x}\n\
        SERIAL_SERIAL_SIOMLT_SEND: {:x}",
                rcnt_str,
                sio_multi_data_0_str,
                sio_multi_data_1_str,
                sio_multi_data_2_str,
                sio_multi_data_3_str,
                sio_cnt_str,
                sio_multi_data_send_str
            )
        }
    }
}

//const SERIAL_LINK: &mut Serial = Serial::new();
//static CURRENT_SAVE_ACCESS: Lock<Option<&'static dyn RawSaveAccess>> = Lock::new(None);

pub struct SerialMultiPlayer<'gba> {
    baudrate: SerialBaudRate,
    is_enabled: bool,
    serial: &'gba mut Serial,
    phantom: PhantomData<&'gba ()>,
}

impl<'gba> SerialMultiPlayer<'gba> {
    fn new(baud_rate: SerialBaudRate) -> SerialMultiPlayer<'gba> {
        let serial = Serial::new();
        SerialMultiPlayer {
            baudrate: baud_rate,
            phantom: PhantomData,
            is_enabled: false,
            serial,
        }
    }

    pub fn activate(&mut self) {
        self.serial.set_multi_player_mode(self.baudrate);
        self.is_enabled = true;
    }
}

impl<'gba> core::fmt::Display for SerialMultiPlayer<'gba> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        self.serial.fmt(f)
    }
}

/// Controls access to the mixer and the underlying hardware it uses. A zero sized type that
/// ensures that mixer access is exclusive.
#[non_exhaustive]
pub struct SerialController {}

impl SerialController {
    pub(crate) const fn new() -> Self {
        SerialController {}
    }

    /// Get a [`SerialMultiPlayer`] in order to start producing sounds.
    pub fn serial_multi_player(&mut self, baud_rate: SerialBaudRate) -> SerialMultiPlayer<'_> {
        SerialMultiPlayer::new(baud_rate)
    }
}
