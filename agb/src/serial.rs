use crate::interrupt::InterruptHandler;
use crate::{timer::Divider, timer::Timer};
use agb::interrupt::{add_interrupt_handler, Interrupt};
use core::cell::RefCell;
use core::marker::PhantomData;
use core::ptr::addr_of;
use core::ptr::addr_of_mut;
use critical_section::Mutex;
use heapless::Deque;

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
const SIO_MULTI_PLAY_OFFLINE_DATA: u16 = 0xFFFF;
const SIO_MULTI_PLAY_EMPTY_DATA: u16 = 0x0;
const SERIAL_MAX_PLAYERS: usize = 4;
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
    sio_multi_data_i: [u16; SERIAL_MAX_PLAYERS],
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
    pub sio_player_id: usize,
}

struct Serial {
    serial_reg: &'static mut SerialReg,
    fifo_send: Deque<u16, 16>,
    fifo_received: [Deque<u16, 16>; 4],
    player_id: usize,
    players_online: usize,
}

const SIO_INVALID_PLAYER_ID: usize = 0xDEADBEEF;

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
            sio_data: [SIO_MULTI_PLAY_OFFLINE_DATA; 4],
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
            while !self.is_sending() {}
            while self.is_sending() {}
            if self.is_ready() && !self.is_error() {
                self.get_data(&mut response);
            }
            self.set_data(SIO_MULTI_PLAY_EMPTY_DATA);
        }
        response
    }

    pub fn handle_serial_interrupt(&mut self) -> SerialResponse {
        let mut response = SerialResponse {
            sio_data: [SIO_MULTI_PLAY_OFFLINE_DATA; 4],
            sio_player_id: SIO_INVALID_PLAYER_ID,
        };
        if self.is_ready() && !self.is_error() {
            self.get_data(&mut response);
            self.set_data(SIO_MULTI_PLAY_EMPTY_DATA);
        }
        response
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

    fn get_player_id(&self) -> usize {
        unsafe {
            (*addr_of!(self.sio_cnt) & SERIAL_CNT_BITS_PLAYER_ID_MASK) as usize
                >> SERIAL_CNT_BITS_PLAYER_ID as usize
        }
    }

    fn get_data<'a>(&self, rsp: &'a mut SerialResponse) -> &'a SerialResponse {
        unsafe {
            for i in 0..SERIAL_MAX_PLAYERS {
                rsp.sio_data[i] = *addr_of!(self.sio_multi_data_i[i]);
            }
            rsp.sio_player_id = self.get_player_id();
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
            let sio_multi_data_0_str = *addr_of!(self.sio_multi_data_i[0]);
            let sio_multi_data_1_str = *addr_of!(self.sio_multi_data_i[1]);
            let sio_multi_data_2_str = *addr_of!(self.sio_multi_data_i[2]);
            let sio_multi_data_3_str = *addr_of!(self.sio_multi_data_i[3]);
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
            fifo_send: Deque::<u16, 16>::new(),
            fifo_received: [
                Deque::<u16, 16>::new(),
                Deque::<u16, 16>::new(),
                Deque::<u16, 16>::new(),
                Deque::<u16, 16>::new(),
            ],
            player_id: SIO_INVALID_PLAYER_ID,
            players_online: 0,
        }
    }

    pub fn handle_hblank_multiplay_irq(&mut self) {
        unsafe {
            if !self.serial_reg.multiplay_mode_reg.is_slave()
                && self.serial_reg.multiplay_mode_reg.is_ready()
                && !self.serial_reg.multiplay_mode_reg.is_error()
                && !self.serial_reg.multiplay_mode_reg.is_sending()
            {
                self.serial_reg.multiplay_mode_reg.start_transmission();
            }
        }
    }

    pub fn handle_serial_multiplay_irq(&mut self) {
        let serial_response;
        unsafe {
            serial_response = self.serial_reg.multiplay_mode_reg.handle_serial_interrupt();
        }
        self.players_online = 0;
        self.player_id = serial_response.sio_player_id;
        if self.player_id != SIO_INVALID_PLAYER_ID {
            for i in 0..SERIAL_MAX_PLAYERS {
                let data = serial_response.sio_data[i];
                if data == SIO_MULTI_PLAY_OFFLINE_DATA {
                    break;
                } else if data == SIO_MULTI_PLAY_EMPTY_DATA {
                    self.players_online += 1;
                } else {
                    let _ = self.fifo_received[i].push_back(data);
                    self.players_online += 1;
                }
            }
            unsafe {
                let data = self
                    .fifo_send
                    .pop_front()
                    .unwrap_or(SIO_MULTI_PLAY_EMPTY_DATA);
                self.serial_reg.multiplay_mode_reg.set_data(data);
            }
        } else {
            agb::println!("\n===ERROR in handle_serial_multiplay_irq===\n");
        }
    }
}

static SERIAL_LINK: Mutex<RefCell<Option<Serial>>> = Mutex::new(RefCell::new(None));

pub struct SerialMultiPlayer<'gba> {
    interrupt_timer: Timer,
    _interrupt_handler: InterruptHandler,
    _interrupt_handler_hblank: InterruptHandler,
    baudrate: SerialBaudRate,
    fifo_send: Deque<u16, 16>,
    fifo_received: [Deque<u16, 16>; 4],
    players_online: usize,
    is_master: bool,
    is_enabled: bool,
    player_id: usize,
    phantom: PhantomData<&'gba ()>,
}

impl<'gba> SerialMultiPlayer<'gba> {
    fn new(baud_rate: SerialBaudRate) -> SerialMultiPlayer<'gba> {
        critical_section::with(|cs| SERIAL_LINK.borrow(cs).replace(Some(Serial::new())));
        let mut interrupt_timer = unsafe { Timer::new(3) };
        interrupt_timer
            .set_cascade(false)
            .set_divider(Divider::Divider64)
            .set_interrupt(true)
            .set_overflow_amount(0x3ff as u16);
        let interrupt_handler = unsafe {
            add_interrupt_handler(interrupt_timer.interrupt(), move |cs| {
                if let Some(ref mut serial) = *SERIAL_LINK.borrow_ref_mut(cs) {
                    serial.handle_hblank_multiplay_irq();
                }
            })
        };
        SerialMultiPlayer {
            interrupt_timer,
            _interrupt_handler: unsafe {
                add_interrupt_handler(Interrupt::Serial, move |cs| {
                    if let Some(ref mut serial) = *SERIAL_LINK.borrow_ref_mut(cs) {
                        serial.handle_serial_multiplay_irq();
                    }
                })
            },
            _interrupt_handler_hblank: interrupt_handler,
            fifo_send: Deque::<u16, 16>::new(),
            fifo_received: [
                Deque::<u16, 16>::new(),
                Deque::<u16, 16>::new(),
                Deque::<u16, 16>::new(),
                Deque::<u16, 16>::new(),
            ],
            baudrate: baud_rate,
            is_master: false,
            phantom: PhantomData,
            is_enabled: false,
            players_online: 0,
            player_id: SIO_INVALID_PLAYER_ID,
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
                    serial.serial_reg.multiplay_mode_reg.enable_interrupt();
                    self.is_master = !serial.serial_reg.multiplay_mode_reg.is_slave();
                    self.player_id = if self.is_master {
                        0
                    } else {
                        SIO_INVALID_PLAYER_ID
                    };
                    self.is_enabled = true;
                    self.interrupt_timer.set_enabled(self.is_master);
                }
            }
        });
    }

    pub fn deactivate(&mut self) {
        critical_section::with(|cs| {
            if let Some(ref mut serial) = *SERIAL_LINK.borrow_ref_mut(cs) {
                unsafe {
                    serial.serial_reg.multiplay_mode_reg.disable_interrupt();
                    serial.serial_reg.gpio_mode_reg.set_gpio_mode();
                }
                self.is_enabled = false;
                self.interrupt_timer.set_enabled(false);
            }
        });
    }

    pub fn is_active(&self) -> bool {
        self.is_enabled
    }

    pub fn sync(&mut self) {
        if self.is_enabled {
            critical_section::with(|cs| {
                if let Some(ref mut serial) = *SERIAL_LINK.borrow_ref_mut(cs) {
                    self.players_online = serial.players_online;
                    self.player_id = serial.player_id;
                    for i in 0..SERIAL_MAX_PLAYERS {
                        while !serial.fifo_received[i].is_empty() {
                            let data = serial.fifo_received[i].pop_front();
                            let _ = self.fifo_received[i].push_back(data.unwrap());
                        }
                    }
                    while !self.fifo_send.is_empty() {
                        if serial.fifo_send.is_full() {
                            agb::println!("\n===ERROR sync!! serial.fifo_send.is_full===\n");
                        }
                        let _ = serial
                            .fifo_send
                            .push_back(self.fifo_send.pop_front().unwrap());
                    }
                }
            });
        }
    }

    pub fn transmit_data(&mut self, data: u16, blocking: bool) -> SerialResponse {
        let mut response = SerialResponse {
            sio_data: [SIO_MULTI_PLAY_OFFLINE_DATA; 4],
            sio_player_id: SIO_INVALID_PLAYER_ID,
        };
        if self.is_enabled {
            critical_section::with(|cs| {
                if let Some(ref mut serial) = *SERIAL_LINK.borrow_ref_mut(cs) {
                    response = unsafe {
                        serial
                            .serial_reg
                            .multiplay_mode_reg
                            .transmit_data(data, blocking)
                    };
                }
            });
        }
        response
    }

    pub fn is_online(&self, player_id: usize) -> bool {
        if self.is_enabled {
            self.players_online <= player_id
        } else {
            false
        }
    }

    pub fn get_nb_players_online(&self) -> usize {
        self.players_online
    }

    pub fn send_data(&mut self, data: u16) {
        let _ = self.fifo_send.push_back(data);
    }

    pub fn get_player_rsp(&mut self, player_id: usize) -> Option<u16> {
        self.fifo_received[player_id].pop_front()
    }

    pub fn get_player_id(&self) -> usize {
        self.player_id
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
