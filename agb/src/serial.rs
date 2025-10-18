use core::marker::PhantomData;
use core::ptr::addr_of;
use core::ptr::addr_of_mut;
use critical_section::CriticalSection;
use crate::interrupt::InterruptHandler;
use agb::interrupt::{add_interrupt_handler, Interrupt};

/// 4000134h - RCNT (R) - Mode Selection, in Normal/Multiplayer/UART modes (R/W)
const SERIAL_RCNT: *mut u16 = (0x04000134) as *mut u16;
const SERIAL_RCNT_BIT_GPIO_L: u16 = 14;
const SERIAL_RCNT_BIT_GPIO_H: u16 = 15;
const SERIAL_BASE_ADDR: usize = 0x04000120;

/// Serial multi-player flags -------------------------------------------
const SERIAL_CNT_BIT_SLAVE: u16 = 2;
const SERIAL_CNT_BIT_CHILD_READY: u16 = 3;
const SERIAL_CNT_BITS_PLAYER_ID: u16 = 4;
const SERIAL_CNT_BITS_PLAYER_ID_MASK: u16 = 3 << SERIAL_CNT_BITS_PLAYER_ID;
const SERIAL_CNT_BIT_ERROR: u16 = 6;
const SERIAL_CNT_BIT_START: u16 = 7;
const SERIAL_CNT_BIT_MULTIPLAYER: u16 = 13;
const SERIAL_CNT_BIT_IRQ: u16 = 14;
const SIO_MULTI_PLAY_EMPTY_DATA: u16 = 0xFFFF;

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

#[derive(Debug, Clone, Copy)]
pub struct SerialResponse {
    pub sio_data: [u16; 4],
    pub sio_player_id: u32,
}

const SIO_INVALID_PLAYER_ID: u32 = 0xDEADBEEF;

impl Serial {
    fn new() -> &'static mut Serial {
        unsafe { &mut *(SERIAL_BASE_ADDR as *mut Self) }
    }

    fn set_multi_player_mode(&mut self, baud_rate: SerialBaudRate) {
        unsafe {
            SERIAL_RCNT
                .write_volatile(SERIAL_RCNT.read_volatile() & !(1 << SERIAL_RCNT_BIT_GPIO_H));
            *addr_of_mut!(self.multiplay_mode_reg.sio_cnt) =
                (1 << SERIAL_CNT_BIT_MULTIPLAYER) | (baud_rate.discriminant());
            *addr_of_mut!(self.multiplay_mode_reg.sio_multi_data_send) = 0;
        }
    }

    fn set_gpio_mode(&mut self) {
        unsafe {
            SERIAL_RCNT.write_volatile(
                (SERIAL_RCNT.read_volatile() & !(1 << SERIAL_RCNT_BIT_GPIO_L))
                    | (1 << SERIAL_RCNT_BIT_GPIO_H),
            );
        }
    }

    #[inline(always)]
    fn wait_end_transmission(&self) {
        while self.is_sending() {}
    }

    #[inline(always)]
    fn set_data(&mut self, data: u16) {
        unsafe {
            *addr_of_mut!(self.multiplay_mode_reg.sio_multi_data_send) = data;
        }
    }

    #[inline(always)]
    fn enable_interrupt(&mut self) {
        unsafe {
            *addr_of_mut!(self.multiplay_mode_reg.sio_cnt) |= 1 << SERIAL_CNT_BIT_IRQ;
        }
    }

    #[inline(always)]
    fn disable_interrupt(&mut self) {
        unsafe {
            *addr_of_mut!(self.multiplay_mode_reg.sio_cnt) &= !(1 << SERIAL_CNT_BIT_IRQ);
        }
    }

    #[inline(always)]
    fn is_sending(&self) -> bool {
        unsafe {
            (addr_of!(self.multiplay_mode_reg.sio_cnt).read_volatile()
                & (1 << SERIAL_CNT_BIT_START))
                != 0
        }
    }

    #[inline(always)]
    fn is_ready(&self) -> bool {
        unsafe {
            (*addr_of!(self.multiplay_mode_reg.sio_cnt) & (1 << SERIAL_CNT_BIT_CHILD_READY)) != 0
        }
    }

    #[inline(always)]
    fn is_slave(&self) -> bool {
        unsafe { (*addr_of!(self.multiplay_mode_reg.sio_cnt) & (1 << SERIAL_CNT_BIT_SLAVE)) != 0 }
    }

    #[inline(always)]
    fn is_error(&self) -> bool {
        unsafe { (*addr_of!(self.multiplay_mode_reg.sio_cnt) & (1 & SERIAL_CNT_BIT_ERROR)) != 0 }
    }

    fn get_data<'a>(&self, rsp: &'a mut SerialResponse) -> &'a SerialResponse {
        unsafe {
            rsp.sio_data[0] = *addr_of!(self.multiplay_mode_reg.sio_multi_data_0);
            rsp.sio_data[1] = *addr_of!(self.multiplay_mode_reg.sio_multi_data_1);
            rsp.sio_data[2] = *addr_of!(self.multiplay_mode_reg.sio_multi_data_2);
            rsp.sio_data[3] = *addr_of!(self.multiplay_mode_reg.sio_multi_data_3);
            rsp.sio_player_id = (*addr_of!(self.multiplay_mode_reg.sio_cnt) as u32
                & SERIAL_CNT_BITS_PLAYER_ID_MASK as u32)
                >> SERIAL_CNT_BITS_PLAYER_ID as u32;
        }
        rsp
    }

    #[inline(always)]
    fn start_transmission(&mut self) {
        unsafe {
            *addr_of_mut!(self.multiplay_mode_reg.sio_cnt) |= 1 << SERIAL_CNT_BIT_START;
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
    _interrupt_handler: InterruptHandler,
    baudrate: SerialBaudRate,
    response: SerialResponse,
    is_master: bool,
    is_enabled: bool,
    serial: &'gba mut Serial,
    phantom: PhantomData<&'gba ()>,
}

impl<'gba> SerialMultiPlayer<'gba> {
    fn new(baud_rate: SerialBaudRate) -> SerialMultiPlayer<'gba> {
        let serial = Serial::new();
        SerialMultiPlayer {
            _interrupt_handler: unsafe {
                     add_interrupt_handler(Interrupt::Serial, |_: CriticalSection| {
                         agb::println!("Woah there! There's been a serial irq!\n");
                     })
                 },
            baudrate: baud_rate,
            is_master: false,
            phantom: PhantomData,
            is_enabled: false,
            response: SerialResponse {
                sio_data: [SIO_MULTI_PLAY_EMPTY_DATA; 4],
                sio_player_id: SIO_INVALID_PLAYER_ID,
            },
            serial,
        }
    }

    pub fn activate(&mut self) {
        self.serial.set_multi_player_mode(self.baudrate);
        self.is_enabled = true;
        self.is_master = !self.serial.is_slave();
    }

    pub fn deactivate(&mut self) {
        self.serial.set_gpio_mode();
        self.is_enabled = false;
    }

    pub fn is_active(&self) -> bool {
        self.is_enabled
    }

    pub fn transmit_data(&mut self, data: u16, blocking: bool) -> SerialResponse {
        self.response = SerialResponse {
            sio_data: [SIO_MULTI_PLAY_EMPTY_DATA; 4],
            sio_player_id: SIO_INVALID_PLAYER_ID,
        };

        self.serial.set_data(data);
        if blocking {
            // test to trig the Irq handler
            self.serial.enable_interrupt();
        } else {
            self.serial.enable_interrupt();
        }
        if self.is_master {
            self.serial.start_transmission();
        }
        if blocking {
            if self.serial.is_ready() && !self.serial.is_error() {
                self.serial.get_data(&mut self.response);
            }
            self.serial.set_data(SIO_MULTI_PLAY_EMPTY_DATA);
        }
        self.response
    }

    pub fn handle_serial_interrupt(&mut self) {
        self.serial.disable_interrupt();
        if self.serial.is_ready() && !self.serial.is_error() {
            self.serial.get_data(&mut self.response);
        }
        self.serial.set_data(SIO_MULTI_PLAY_EMPTY_DATA);
    }

    pub fn is_online(&self, player_id: u32) -> bool {
        self.response.sio_data[player_id as usize] == SIO_MULTI_PLAY_EMPTY_DATA
    }

    pub fn get_response(&mut self, player_id: u32) -> u16 {
        self.response.sio_data[player_id as usize]
    }

    pub fn get_player_id(&self) -> u32 {
        self.response.sio_player_id
    }
}

impl<'gba> core::fmt::Display for SerialMultiPlayer<'gba> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        self.serial.fmt(f)
    }
}

/// Controls access to the serial and the underlying hardware it uses. A zero sized type that
/// ensures that serial access is exclusive.
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
