use crate::interrupt::InterruptHandler;
use crate::{timer::Divider, timer::Timer};
use agb::interrupt::{add_interrupt_handler, Interrupt};
use bitflags::bitflags;
use core::cell::RefCell;
use core::marker::PhantomData;
use core::ptr::addr_of;
use core::ptr::addr_of_mut;
use critical_section::Mutex;
use heapless::Deque;

/// 4000134h - RCNT (R) - Mode Selection, in Normal/Multiplayer/UART modes (R/W)
const SERIAL_RCNT: *mut u16 = (0x04000134) as *mut u16;
bitflags! {
    /// Describe the SERIAL_RCNT bitfields
    #[derive(PartialEq, Eq, Hash, Debug, Clone, Copy)]
    pub struct SerialRcnt: u16 {
        /// GPIO mode Low reg
        const SERIAL_RCNT_BIT_GPIO_L = 1 << 14;
        /// GPIO mode High reg
        const SERIAL_RCNT_BIT_GPIO_H = 1 << 15;
    }
}
///4000120h - SIO - Serial Link IP
const SERIAL_BASE_ADDR: usize = 0x04000120;
bitflags! {
    #[derive(PartialEq, Eq, Hash, Debug, Clone, Copy)]
    /// Describe the SIO_CNT bitfields
    pub struct SerialCnt: u16 {
        /// Is slave if set (RO)
        const SERIAL_CNT_BIT_SLAVE = 1 << 2;
        /// All slaves ready if set (RO)
        const SERIAL_CNT_BIT_CHILD_READY = 1 << 3;
        /// Player id (bits 4 RO)
        const SERIAL_CNT_BITS_PLAYER_ID0 = 1 << 4;
        /// Player id (bits 5 RO)
        const SERIAL_CNT_BITS_PLAYER_ID1 = 1 << 5;
        /// An error occured if set (RO)
        const SERIAL_CNT_BIT_ERROR = 1 << 6;
        /// Set this bit to start a transaction. If set line is busy. (master RW, slave RO)
        const SERIAL_CNT_BIT_START = 1 << 7;
        /// Multiplay mode enable/disable
        const SERIAL_CNT_BIT_MULTIPLAYER = 1 << 13;
        /// Enable / Disable Serial IRQ
        const SERIAL_CNT_BIT_IRQ = 1 << 14;
    }
}

/// index where is player id in SIO_CNT register.
const SERIAL_CNT_BITS_PLAYER_BIT_IDX: usize = 4;
const SERIAL_CNT_BITS_PLAYER_ID_MASK: u16 =
    SerialCnt::SERIAL_CNT_BITS_PLAYER_ID0.bits() | SerialCnt::SERIAL_CNT_BITS_PLAYER_ID1.bits();
/// When cable not connected, 0xFFFF is read in sio_multi_data registers.
const SIO_MULTI_PLAY_OFFLINE_DATA: u16 = 0xFFFF;
/// When nothing to send, 0x0000 is sent.
/// Because slave can't notify when they have data to send,
/// the master need to poll slaves and send empty data when we have nothing to send.
const SIO_MULTI_PLAY_EMPTY_DATA: u16 = 0x0;
/// Serial Multiplay mode can handle up to 4 players (master included).
const SERIAL_MAX_PLAYERS: usize = 4;

/// BaudRates supported by the serial link
#[repr(u16)]
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Clone, Copy)]
pub enum SerialBaudRate {
    /// BaudRate 9600 bps
    BaudRate0 = 0,
    /// BaudRate 38400 bps
    BaudRate1 = 1,
    /// BaudRate 57600 bps
    BaudRate2 = 2,
    /// BaudRate 115200 bps
    BaudRate3 = 3,
}

impl SerialBaudRate {
    /// Get the u16 value from the enum.
    fn discriminant(&self) -> u16 {
        unsafe { *(self as *const Self as *const u16) }
    }
}

#[derive(Clone, Copy)]
#[repr(C, packed)]
/// Registers mapping in Multi-Player mode.
struct SerialMultiPlayerReg {
    /// [4000120h - 4000126] - SIOMULTI[0-3] - SIO Multi-Player Data [0-3] (Parent) (R/W)
    sio_multi_data_i: [u16; SERIAL_MAX_PLAYERS],
    /// 4000128h - SIOCNT - SIO Control, usage in MULTI-PLAYER Mode (R/W)
    sio_cnt: u16,
    /// 400012Ah - SIOMLT_SEND - Data Send Register (R/W)
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

/// Internal structure used to received data in multiplay mode.
#[derive(Debug, Clone, Copy)]
pub struct SerialResponse {
    /// received data from sio_multi_data regs
    pub sio_data: [u16; 4],
    /// player id extracted from the register SIO_CNT
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
                (SERIAL_RCNT.read_volatile() & !SerialRcnt::SERIAL_RCNT_BIT_GPIO_L.bits())
                    | SerialRcnt::SERIAL_RCNT_BIT_GPIO_H.bits(),
            );
        }
    }
}

impl SerialMultiPlayerReg {
    /// enable the serial link in multiplay mode.
    fn set_multi_player_mode(&mut self, baud_rate: SerialBaudRate) {
        unsafe {
            SERIAL_RCNT.write_volatile(
                SERIAL_RCNT.read_volatile() & !SerialRcnt::SERIAL_RCNT_BIT_GPIO_H.bits(),
            );
            *addr_of_mut!(self.sio_cnt) =
                SerialCnt::SERIAL_CNT_BIT_MULTIPLAYER.bits() | baud_rate.discriminant();
            *addr_of_mut!(self.sio_multi_data_send) = 0;
        }
    }

    #[inline(always)]
    /// set multiplay data to send for the next start
    fn set_data(&mut self, data: u16) {
        unsafe {
            *addr_of_mut!(self.sio_multi_data_send) = data;
        }
    }

    #[inline(always)]
    /// enable serial irq
    fn enable_interrupt(&mut self) {
        unsafe {
            *addr_of_mut!(self.sio_cnt) |= SerialCnt::SERIAL_CNT_BIT_IRQ.bits();
        }
    }

    #[inline(always)]
    /// disable serial irq
    fn disable_interrupt(&mut self) {
        unsafe {
            *addr_of_mut!(self.sio_cnt) &= !SerialCnt::SERIAL_CNT_BIT_IRQ.bits();
        }
    }

    #[inline(always)]
    #[must_use]
    /// Returns `true` if a transmission is on going, and `false` if not.
    fn is_sending(&self) -> bool {
        unsafe {
            (addr_of!(self.sio_cnt).read_volatile() & SerialCnt::SERIAL_CNT_BIT_START.bits()) != 0
        }
    }

    #[inline(always)]
    #[must_use]
    /// Returns `true` if all slaves are connected in multiplay mode, and `false` if not.
    fn is_ready(&self) -> bool {
        unsafe { (*addr_of!(self.sio_cnt) & SerialCnt::SERIAL_CNT_BIT_CHILD_READY.bits()) != 0 }
    }

    #[inline(always)]
    #[must_use]
    /// Returns `true` if connected as slave, and `false` if connected as master.
    fn is_slave(&self) -> bool {
        unsafe { (*addr_of!(self.sio_cnt) & SerialCnt::SERIAL_CNT_BIT_SLAVE.bits()) != 0 }
    }

    #[inline(always)]
    #[must_use]
    /// Returns `true` if an error occurred, and `false` if not.
    fn is_error(&self) -> bool {
        unsafe { (*addr_of!(self.sio_cnt) & SerialCnt::SERIAL_CNT_BIT_ERROR.bits()) != 0 }
    }

    #[inline(always)]
    #[must_use]
    /// Returns player id between 0-3.
    fn get_player_id(&self) -> usize {
        unsafe {
            (*addr_of!(self.sio_cnt) & SERIAL_CNT_BITS_PLAYER_ID_MASK) as usize
                >> SERIAL_CNT_BITS_PLAYER_BIT_IDX as usize
        }
    }

    /// Returns `SerialResponse` of the last transfer.
    fn get_data<'a>(&self, rsp: &'a mut SerialResponse) {
        for i in 0..SERIAL_MAX_PLAYERS {
            unsafe {
                rsp.sio_data[i] = *addr_of!(self.sio_multi_data_i[i]);
            }
        }
        rsp.sio_player_id = self.get_player_id();
    }

    #[inline(always)]
    /// Start transmission. Must be done by the master.
    fn start_transmission(&mut self) {
        unsafe {
            *addr_of_mut!(self.sio_cnt) |= SerialCnt::SERIAL_CNT_BIT_START.bits();
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
        let mut response = SerialResponse {
            sio_data: [SIO_MULTI_PLAY_OFFLINE_DATA; 4],
            sio_player_id: SIO_INVALID_PLAYER_ID,
        };
        unsafe {
            if self.serial_reg.multiplay_mode_reg.is_ready()
                && !self.serial_reg.multiplay_mode_reg.is_error()
                && !self.serial_reg.multiplay_mode_reg.is_sending()
            {
                self.serial_reg.multiplay_mode_reg.get_data(&mut response);
                self.serial_reg
                    .multiplay_mode_reg
                    .set_data(SIO_MULTI_PLAY_EMPTY_DATA);
            }
        }
        self.players_online = 0;
        self.player_id = response.sio_player_id;
        if self.player_id != SIO_INVALID_PLAYER_ID {
            for i in 0..SERIAL_MAX_PLAYERS {
                let data = response.sio_data[i];
                if data == SIO_MULTI_PLAY_OFFLINE_DATA {
                    break;
                } else if data == SIO_MULTI_PLAY_EMPTY_DATA {
                    self.players_online += 1;
                } else {
                    let _ = self.fifo_received[i].push_back(data);
                    self.players_online += 1;
                }
            }
            let data = self
                .fifo_send
                .pop_front()
                .unwrap_or(SIO_MULTI_PLAY_EMPTY_DATA);
            unsafe {
                self.serial_reg.multiplay_mode_reg.set_data(data);
            }
        } else {
            agb::println!("\n===ERROR in handle_serial_multiplay_irq===\n");
        }
    }
}

/// Serial link access
static SERIAL_LINK: Mutex<RefCell<Option<Serial>>> = Mutex::new(RefCell::new(None));

/// The main software Serial multiplayer struct.
///
/// Handle async communication between all connected GBA.
/// The master is in charge of starting the communication. The master start a communication for each Timer IRQ to poll slaves.
/// This way, all connected players can receive and send data.
/// All connected slaves store received data on each Serial IRQ and set the next data to send.
/// You should not create this struct directly, instead creating it through the [`Gba`](crate::Gba)
/// struct as follows:
/// ```rust
/// # #![no_std]
/// # #![no_main]
/// # use agb::*;
/// # #[agb::doctest]
/// # fn test(mut gba: Gba) {
/// /// example to do.
/// # }
/// ```
///
pub struct SerialMultiPlayer<'gba> {
    interrupt_timer: Timer,
    _interrupt_handler: InterruptHandler,
    _interrupt_handler_timer: InterruptHandler,
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
        let mut interrupt_timer = unsafe { Timer::new(2) };
        interrupt_timer
            .set_cascade(false)
            .set_divider(Divider::Divider64)
            .set_interrupt(true)
            .set_overflow_amount(0x1FF as u16);
        let interrupt_handler = unsafe {
            add_interrupt_handler(interrupt_timer.interrupt(), move |cs| {
                if let Some(ref mut serial) = *SERIAL_LINK.borrow_ref_mut(cs) {
                    serial.handle_hblank_multiplay_irq();
                }
                agb::println!("\n===timer3 irq occurred===\n");
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
            _interrupt_handler_timer: interrupt_handler,
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

    /// Activate the multiplay mode.
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

    /// Deactivate the multiplay mode.
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

    /// Return `true` if the multiplay mode is activated, and `false` if not.
    pub fn is_active(&self) -> bool {
        self.is_enabled
    }

    /// Sync data received and to send between user and Irq.
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

    /// Return True if the player `player_id` is connected.
    pub fn is_online(&self, player_id: usize) -> bool {
        if self.is_enabled {
            self.players_online <= player_id
        } else {
            false
        }
    }

    /// Return the number of player connected since the last sync.
    pub fn get_nb_players_online(&self) -> usize {
        self.players_online
    }

    /// Push in the fifo a data to send for the next sync.
    pub fn send_data(&mut self, data: u16) {
        let _ = self.fifo_send.push_back(data);
    }

    /// Pop response from `player_id`.
    pub fn get_player_rsp(&mut self, player_id: usize) -> Option<u16> {
        self.fifo_received[player_id].pop_front()
    }

    /// Return your `player_id`.
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

    /// Get a [`SerialMultiPlayer`] in order to control the Serial interface.
    pub fn serial_multi_player(&mut self, baud_rate: SerialBaudRate) -> SerialMultiPlayer<'_> {
        SerialMultiPlayer::new(baud_rate)
    }
}
