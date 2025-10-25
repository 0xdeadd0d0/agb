use crate::interrupt::InterruptHandler;
use agb::interrupt::{add_interrupt_handler, Interrupt};
use core::cell::RefCell;
use core::marker::PhantomData;
use core::ptr::addr_of;
use core::ptr::addr_of_mut;
use critical_section::{CriticalSection, Mutex};

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
union SerialReg {
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

struct Serial {
    serial_reg: &'static mut SerialReg,
    to_send: u16,
    serial_response: SerialResponse,
}

const SIO_INVALID_PLAYER_ID: u32 = 0xDEADBEEF;

impl SerialGpio {
    fn set_gpio_mode(&mut self) {
        unsafe {
            SERIAL_RCNT.write_volatile(
                (SERIAL_RCNT.read_volatile() & !(1 << SERIAL_RCNT_BIT_GPIO_L))
                    | (1 << SERIAL_RCNT_BIT_GPIO_H),
            );
        }
    }
}

impl SerialMultiPlayerReg {
    fn set_multi_player_mode(&mut self, baud_rate: SerialBaudRate) {
        unsafe {
            SERIAL_RCNT
                .write_volatile(SERIAL_RCNT.read_volatile() & !(1 << SERIAL_RCNT_BIT_GPIO_H));
            *addr_of_mut!(self.sio_cnt) =
                (1 << SERIAL_CNT_BIT_MULTIPLAYER) | (baud_rate.discriminant());
            *addr_of_mut!(self.sio_multi_data_send) = 0;
        }
    }

    pub fn transmit_data(&mut self, data: u16, blocking: bool) -> SerialResponse {
        let mut response = SerialResponse {
            sio_data: [SIO_MULTI_PLAY_EMPTY_DATA; 4],
            sio_player_id: SIO_INVALID_PLAYER_ID,
        };
        self.set_data(data);
        if blocking {
            // test to trig the Irq handler
            self.disable_interrupt();
        } else {
            self.enable_interrupt();
        }
        if !self.is_slave() {
            self.start_transmission();
        }
        if blocking {
            if self.is_ready() && !self.is_error() {
                self.get_data(&mut response);
            }
            self.set_data(SIO_MULTI_PLAY_EMPTY_DATA);
        }
        response
    }

    pub fn handle_serial_interrupt(&mut self) -> SerialResponse {
        let mut response = SerialResponse {
            sio_data: [SIO_MULTI_PLAY_EMPTY_DATA; 4],
            sio_player_id: SIO_INVALID_PLAYER_ID,
        };
        if self.is_ready() && !self.is_error() {
            self.get_data(&mut response);
        }
        self.set_data(SIO_MULTI_PLAY_EMPTY_DATA);
        response
    }

    #[inline(always)]
    fn wait_end_transmission(&self) {
        while self.is_sending() {}
    }

    #[inline(always)]
    fn set_data(&mut self, data: u16) {
        unsafe {
            *addr_of_mut!(self.sio_multi_data_send) = data;
        }
    }

    #[inline(always)]
    fn enable_interrupt(&mut self) {
        unsafe {
            *addr_of_mut!(self.sio_cnt) |= 1 << SERIAL_CNT_BIT_IRQ;
        }
    }

    #[inline(always)]
    fn disable_interrupt(&mut self) {
        unsafe {
            *addr_of_mut!(self.sio_cnt) &= !(1 << SERIAL_CNT_BIT_IRQ);
        }
    }

    #[inline(always)]
    fn is_sending(&self) -> bool {
        unsafe { (addr_of!(self.sio_cnt).read_volatile() & (1 << SERIAL_CNT_BIT_START)) != 0 }
    }

    #[inline(always)]
    fn is_ready(&self) -> bool {
        unsafe { (*addr_of!(self.sio_cnt) & (1 << SERIAL_CNT_BIT_CHILD_READY)) != 0 }
    }

    #[inline(always)]
    fn is_slave(&self) -> bool {
        unsafe { (*addr_of!(self.sio_cnt) & (1 << SERIAL_CNT_BIT_SLAVE)) != 0 }
    }

    #[inline(always)]
    fn is_error(&self) -> bool {
        unsafe { (*addr_of!(self.sio_cnt) & (1 & SERIAL_CNT_BIT_ERROR)) != 0 }
    }

    fn get_data<'a>(&self, rsp: &'a mut SerialResponse) -> &'a SerialResponse {
        unsafe {
            rsp.sio_data[0] = *addr_of!(self.sio_multi_data_0);
            rsp.sio_data[1] = *addr_of!(self.sio_multi_data_1);
            rsp.sio_data[2] = *addr_of!(self.sio_multi_data_2);
            rsp.sio_data[3] = *addr_of!(self.sio_multi_data_3);
            rsp.sio_player_id = (*addr_of!(self.sio_cnt) as u32
                & SERIAL_CNT_BITS_PLAYER_ID_MASK as u32)
                >> SERIAL_CNT_BITS_PLAYER_ID as u32;
        }
        rsp
    }

    #[inline(always)]
    fn start_transmission(&mut self) {
        unsafe {
            *addr_of_mut!(self.sio_cnt) |= 1 << SERIAL_CNT_BIT_START;
        }
    }
}

impl core::fmt::Display for SerialMultiPlayerReg {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        unsafe {
            let rcnt_str = SERIAL_RCNT.read_volatile();
            let sio_multi_data_0_str = *addr_of!(self.sio_multi_data_0);
            let sio_multi_data_1_str = *addr_of!(self.sio_multi_data_1);
            let sio_multi_data_2_str = *addr_of!(self.sio_multi_data_2);
            let sio_multi_data_3_str = *addr_of!(self.sio_multi_data_3);
            let sio_cnt_str = *addr_of!(self.sio_cnt);
            let sio_multi_data_send_str = *addr_of!(self.sio_multi_data_send);
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

impl Serial {
    fn new() -> Serial {
        Serial {
            serial_reg: unsafe { &mut *(SERIAL_BASE_ADDR as *mut SerialReg) },
            to_send: SIO_MULTI_PLAY_EMPTY_DATA,
            serial_response: SerialResponse {
                sio_data: [SIO_MULTI_PLAY_EMPTY_DATA; 4],
                sio_player_id: SIO_INVALID_PLAYER_ID,
            },
        }
    }

    pub fn handle_serial_multiplay_irq(&mut self) {
        unsafe {
            self.serial_response = self.serial_reg.multiplay_mode_reg.handle_serial_interrupt();
        }
    }
}

static SERIAL_LINK: Mutex<RefCell<Option<Serial>>> = Mutex::new(RefCell::new(None));

pub struct SerialMultiPlayer<'gba> {
    _interrupt_handler: InterruptHandler,
    baudrate: SerialBaudRate,
    response: SerialResponse,
    is_master: bool,
    is_enabled: bool,
    phantom: PhantomData<&'gba ()>,
}

impl<'gba> SerialMultiPlayer<'gba> {
    fn new(baud_rate: SerialBaudRate) -> SerialMultiPlayer<'gba> {
        critical_section::with(|cs| SERIAL_LINK.borrow(cs).replace(Some(Serial::new())));
        SerialMultiPlayer {
            _interrupt_handler: unsafe {
                add_interrupt_handler(Interrupt::Serial, move |cs| {
                    if let Some(ref mut serial) = *SERIAL_LINK.borrow_ref_mut(cs) {
                        serial.handle_serial_multiplay_irq();
                        agb::println!("{}", serial.serial_reg.multiplay_mode_reg);
                    }
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
        }
    }

    pub fn activate(&mut self) {
        critical_section::with(|cs| {
            if let Some(ref mut serial) = *SERIAL_LINK.borrow_ref_mut(cs) {
                unsafe {
                    serial
                        .serial_reg
                        .multiplay_mode_reg
                        .set_multi_player_mode(self.baudrate);
                    self.is_master = !serial.serial_reg.multiplay_mode_reg.is_slave();
                }
            }
        });
        self.is_enabled = true;
    }

    pub fn deactivate(&mut self) {
        critical_section::with(|cs| {
            if let Some(ref mut serial) = *SERIAL_LINK.borrow_ref_mut(cs) {
                unsafe {
                    serial.serial_reg.gpio_mode_reg.set_gpio_mode();
                }
            }
        });
        self.is_enabled = false;
    }

    pub fn is_active(&self) -> bool {
        self.is_enabled
    }

    pub fn sync(&mut self) {
        critical_section::with(|cs| {
            if let Some(ref mut serial) = *SERIAL_LINK.borrow_ref_mut(cs) {
                self.response = unsafe { serial.serial_response };
            }
        });
    }

    pub fn transmit_data(&mut self, data: u16, blocking: bool) -> SerialResponse {
        critical_section::with(|cs| {
            if let Some(ref mut serial) = *SERIAL_LINK.borrow_ref_mut(cs) {
                self.response = unsafe {
                    serial
                        .serial_reg
                        .multiplay_mode_reg
                        .transmit_data(data, blocking)
                }
            }
        });
        self.response
    }

    pub fn is_online(&self, player_id: u32) -> bool {
        self.response.sio_data[player_id as usize] != SIO_MULTI_PLAY_EMPTY_DATA
    }

    pub fn get_response(&mut self) -> SerialResponse {
        self.response
    }

    pub fn get_player_id(&self) -> u32 {
        self.response.sio_player_id
    }
}

impl<'gba> core::fmt::Display for SerialMultiPlayer<'gba> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        critical_section::with(|cs| {
            if let Some(ref mut serial) = *SERIAL_LINK.borrow_ref_mut(cs) {
                unsafe { serial.serial_reg.multiplay_mode_reg.fmt(f) }
            } else {
                write!(f, "")
            }
        })
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
