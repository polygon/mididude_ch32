use ch32_hal::gpio::Pin;
use ch32_hal::interrupt::typelevel::{Binding, Interrupt};
use ch32_hal::pac::gpio::vals::{Cnf, Mode};
use ch32_hal::time::Hertz;
use ch32_hal::{interrupt, into_ref, peripherals};
use ch32_hal::{pac::gpio::Gpio, Peripheral, Peripherals};
use core::fmt::Write;
use core::future;
use core::marker::PhantomData;
use core::task::Poll;
use embassy_sync::waitqueue::AtomicWaker;

/// Event interrupt handler.
pub struct EventInterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::EventInterrupt> for EventInterruptHandler<T> {
    unsafe fn on_interrupt() {
        on_interrupt::<T>()
    }
}

/// Error interrupt handler.
pub struct ErrorInterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::ErrorInterrupt> for ErrorInterruptHandler<T> {
    unsafe fn on_interrupt() {
        on_interrupt::<T>()
    }
}

pub unsafe fn on_interrupt<T: Instance>() {
    let regs = T::regs();
    // i2c v2 only woke the task on transfer complete interrupts. v1 uses interrupts for a bunch of
    // other stuff, so we wake the task on every interrupt.
    T::state().waker.wake();
    critical_section::with(|_| {
        // Clear event interrupt flag.
        regs.ctlr2().modify(|w| {
            w.set_itevten(false);
            w.set_iterren(false);
        });
    });
}

// I2C error
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
// TODO: Reinstate eventually
//#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(defmt::Format)]
#[non_exhaustive]
pub enum Error {
    /// I2C abort with error
    Abort,
    /// User passed in a response buffer that was 0 length
    InvalidResponseBufferLength,
    /// The response buffer length was too short to contain the message
    ///
    /// The length parameter will always be the length of the buffer, and is
    /// provided as a convenience for matching alongside `Command::Write`.
    /// Second returned value is total size of this write
    PartialWrite(usize, usize),
    /// The response buffer length was too short to contain the message
    ///
    /// The length parameter will always be the length of the buffer, and is
    /// provided as a convenience for matching alongside `Command::GeneralCall`.
    PartialGeneralCall(usize),
}

/// Received command
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
// TODO: Reinstate eventually
//#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(defmt::Format)]
pub enum Command {
    /// General Call
    GeneralCall(usize),
    /// Read
    Read,
    /// Write+read
    WriteRead(usize),
    /// Write
    Write(usize),
}

/// Possible responses to responding to a read
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
// TODO: Reinstate eventually
//#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(defmt::Format)]
pub enum ReadStatus {
    /// Transaction Complete, controller naked our last byte
    Done,
    /// Transaction Incomplete, controller trying to read more bytes than were provided
    NeedMoreBytes,
    /// Transaction Complere, but controller stopped reading bytes before we ran out
    LeftoverBytes(u16),
}

/// Slave Configuration
#[non_exhaustive]
#[derive(Copy, Clone)]
// TODO: Reinstate eventually
//#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(defmt::Format)]
pub struct Config {
    /// Target Address
    pub addr: u8,
    /// Control if the peripheral should ack to and report general calls.
    pub general_call: bool,
    // Frequency
    pub freq: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            addr: 0x55,
            general_call: true,
            freq: 48000000,
        }
    }
}

/// I2CSlave driver.
pub struct I2cSlave<'d, T: Instance> {
    phantom: PhantomData<&'d mut T>,
    config: Config,
}

#[derive(Debug, Clone)]
enum TransactionState {
    Idle,
    Undecided,
    Receiving,
}

static mut glob_sta: TransactionState = TransactionState::Idle;

impl<'d, T: Instance> I2cSlave<'d, T> {
    /// Create a new instance.
    pub fn new<const REMAP: u8>(
        _peri: impl Peripheral<P = T> + 'd,
        config: Config,
        sda_pin: impl SdaPin<T, REMAP>,
        scl_pin: impl SclPin<T, REMAP>,
        _irq: impl interrupt::typelevel::Binding<T::EventInterrupt, EventInterruptHandler<T>>
            + interrupt::typelevel::Binding<T::ErrorInterrupt, ErrorInterruptHandler<T>>
            + 'd,
    ) -> Self {
        assert!(config.addr != 0);

        // Configure pin mapping
        let afio = ch32_hal::pac::AFIO;
        afio.pcfr1().modify(|r| {
            r.set_i2c1_rm(REMAP & 0x1 > 0);
            r.set_i2c1_rm1(REMAP & 0x2 > 0);
        });

        // Configure Pins
        ch32_hal::pac::GPIO(sda_pin.port() as usize)
            .cfglr()
            .modify(|r| {
                r.set_mode(sda_pin.pin() as usize, Mode::OUTPUT_50MHZ);
                r.set_cnf(sda_pin.pin() as usize, Cnf::AF_OPEN_DRAIN_OUT);
            });
        ch32_hal::pac::GPIO(scl_pin.port() as usize)
            .cfglr()
            .modify(|r| {
                r.set_mode(scl_pin.pin() as usize, Mode::OUTPUT_50MHZ);
                r.set_cnf(scl_pin.pin() as usize, Cnf::AF_OPEN_DRAIN_OUT);
            });

        // Enable clock and reset module
        let rcc = &ch32_hal::pac::RCC;
        rcc.apb1pcenr().modify(|r| {
            r.set_i2c1en(true);
        });
        rcc.apb1prstr().write(|r| r.set_i2c1rst(true));
        rcc.apb1prstr().write(|r| r.set_i2c1rst(false));

        let mut ret = Self {
            phantom: PhantomData,
            config,
        };

        ret.reset();

        T::EventInterrupt::unpend();
        T::ErrorInterrupt::unpend();
        unsafe {
            T::EventInterrupt::enable();
            T::ErrorInterrupt::enable();
        }

        ret
    }

    pub fn reset(&self) {
        let i2c = T::regs();

        // Disable I2C
        i2c.ctlr1().modify(|w| w.set_pe(false));

        // soft reset
        i2c.ctlr1().modify(|w| w.set_swrst(true));
        i2c.ctlr1().modify(|w| w.set_swrst(false));

        // Set address (currently 7 bit only), currently no general call
        i2c.oaddr1().modify(|r| {
            r.set_add7_1(self.config.addr);
            r.set_addmode(self.config.general_call);
        });

        // Set clock freq
        let freq_set = (self.config.freq / 1000000) as u8;
        assert!(freq_set >= 8);
        assert!(freq_set <= 48);
        i2c.ctlr2().modify(|r| {
            r.set_freq((self.config.freq / 1000000) as u8);
        });

        i2c.ctlr1().modify(|r| {
            r.set_pe(true);
        });

        // Enable ACK
        i2c.ctlr1().modify(|r| {
            r.set_ack(true);
        });
    }

    // #[inline(always)]
    // pub fn get_state(&self) ->

    /// Calls `f` to check if we are ready or not.
    /// If not, `g` is called once(to eg enable the required interrupts).
    /// The waker will always be registered prior to calling `f`.
    #[inline(always)]
    async fn wait_on<F, U>(&mut self, mut f: F) -> U
    where
        F: FnMut(&mut Self) -> Poll<U>,
    {
        future::poll_fn(|cx| {
            // Register prior to checking the condition
            T::state().waker.register(cx.waker());
            let r = f(self);

            if r.is_pending() {
                Self::enable_interrupts();
            }

            r
        })
        .await
    }

    #[inline(always)] // pretty sure this should always be inlined
    fn enable_interrupts() -> () {
        T::regs().ctlr2().modify(|w| {
            w.set_iterren(true);
            w.set_itevten(true);
        });
    }

    pub async fn listen(
        &mut self,
        buf: &mut [u8],
        uart: &mut impl Write,
    ) -> Result<Command, Error> {
        let i2c = T::regs();

        let mut rx_len = 0;
        let mut state = TransactionState::Idle;
        self.wait_on(move |me| {
            let star1 = i2c.star1().read();

            //writeln!(uart, "State: {:?}, star1: 0x{:x}\r", state, star1.0);

            match state {
                TransactionState::Idle | TransactionState::Undecided => {
                    if star1.berr() || star1.af() {
                        i2c.star1().modify(|r| {
                            r.set_berr(false);
                            r.set_af(false);
                        });
                        return Poll::Ready(Err(Error::Abort));
                    }

                    if star1.addr() {
                        // Acknowledge ADDR match
                        i2c.star1().read();
                        i2c.star2().read();
                        state = TransactionState::Undecided;
                    } else {
                        if star1.rx_ne() {
                            if let TransactionState::Undecided = state {
                                state = TransactionState::Receiving;
                            }
                        } else {
                            if star1.stopf() {
                                // Clear STOPF bit
                                let val = i2c.ctlr1().read();
                                i2c.star1().read();
                                i2c.ctlr1().write_value(val);
                            }
                        }
                    }

                    if star1.tx_e() {
                        return Poll::Ready(Ok(Command::Read));
                    }

                    //Poll::Pending
                }
                TransactionState::Receiving => {
                    if star1.rx_ne() {
                        // Another byte available
                        let new_byte = i2c.datar().read().datar();
                        if rx_len < buf.len() {
                            buf[rx_len] = new_byte;
                        }
                        rx_len += 1;
                    }

                    if star1.stopf() {
                        // Clear stop
                        i2c.ctlr1().write_value(i2c.ctlr1().read());
                        if rx_len > buf.len() {
                            return Poll::Ready(Err(Error::PartialWrite(buf.len(), rx_len)));
                        } else {
                            return Poll::Ready(Ok(Command::Write(rx_len)));
                        }
                    }

                    if star1.tx_e() && star1.addr() {
                        if rx_len > buf.len() {
                            return Poll::Ready(Err(Error::PartialWrite(buf.len(), rx_len)));
                        } else {
                            return Poll::Ready(Ok(Command::Write(buf.len())));
                        }
                    }

                    if star1.addr() {
                        // This should not happen, possibly restart
                        writeln!(uart, "BUG: addr seen while rx\r");
                        return Poll::Ready(Ok(Command::Write(rx_len)));
                    }

                    //Poll::Pending
                }
            }
            unsafe {
                glob_sta = state.clone();
            }
            Poll::Pending
        })
        .await
    }
}

struct State {
    #[allow(unused)]
    waker: AtomicWaker,
}

impl State {
    const fn new() -> Self {
        Self {
            waker: AtomicWaker::new(),
        }
    }
}

trait SealedInstance: ch32_hal::RccPeripheral + ch32_hal::RemapPeripheral {
    fn regs() -> ch32_hal::pac::i2c::I2c;
    fn state() -> &'static State;
}

impl SealedInstance for peripherals::I2C1 {
    fn regs() -> ch32_hal::pac::i2c::I2c {
        ch32_hal::pac::I2C1
    }

    fn state() -> &'static State {
        static STATE: State = State::new();
        &STATE
    }
}

impl Instance for peripherals::I2C1 {
    type EventInterrupt = ch32_hal::interrupt::typelevel::I2C1_EV;
    type ErrorInterrupt = ch32_hal::interrupt::typelevel::I2C1_ER;
}

/// I2C peripheral instance
#[allow(private_bounds)]
pub trait Instance: SealedInstance + 'static {
    /// Event interrupt for this instance
    type EventInterrupt: interrupt::typelevel::Interrupt;
    /// Error interrupt for this instance
    type ErrorInterrupt: interrupt::typelevel::Interrupt;
}

pub trait SdaPin<T: Instance, const REMAP: u8 = 0>: ch32_hal::gpio::Pin {}
pub trait SclPin<T: Instance, const REMAP: u8 = 0>: ch32_hal::gpio::Pin {}
impl SdaPin<ch32_hal::peripherals::I2C1, 0> for ch32_hal::peripherals::PC1 {}
impl SclPin<ch32_hal::peripherals::I2C1, 0> for ch32_hal::peripherals::PC2 {}
impl SdaPin<ch32_hal::peripherals::I2C1, 1> for ch32_hal::peripherals::PD0 {}
impl SclPin<ch32_hal::peripherals::I2C1, 1> for ch32_hal::peripherals::PD1 {}
impl SdaPin<ch32_hal::peripherals::I2C1, 2> for ch32_hal::peripherals::PC6 {}
impl SclPin<ch32_hal::peripherals::I2C1, 2> for ch32_hal::peripherals::PC5 {}

pub fn monitor_addr(uart: &mut impl Write) {
    let i2c = &ch32_hal::pac::I2C1;
    i2c.ctlr1().modify(|r| {
        r.set_ack(true);
    });

    let mut last: u32 = 0xaffeaffe;
    loop {
        let next = i2c.star1().read();
        if next.0 != last {
            last = next.0;
            writeln!(uart, "{:#x}\r", last).unwrap();
        }

        if next.af() {
            writeln!(uart, "ACK_ERROR\r");
            i2c.star1().modify(|r| r.set_af(false));
        }

        if next.rx_ne() {
            writeln!(uart, "RCV: {:#x}\r", i2c.datar().read().datar()).unwrap();
            continue;
        }

        if next.tx_e() {
            writeln!(uart, "WR\r").unwrap();
            i2c.datar().write(|r| r.set_datar(0x42));
            continue;
        }

        if next.stopf() {
            writeln!(uart, "STOP\n").unwrap();
            i2c.ctlr1().write_value(i2c.ctlr1().read());
            continue;
        }

        if next.addr() {
            /*writeln!(
                uart,
                "2: {:#x}: {:#x}\r",
                i2c.star2().read().0,
                i2c.datar().read().datar()
            )
            .unwrap();*/
            i2c.star2().read();
            continue;
        }
    }

    let mut star1;
    let mut star2;
    loop {
        /* Do not switch order, reading star1 before star2 will cause ADDR to be cleared and in case of
        transmit we will transmit what is currently inside the data register */
        star2 = i2c.star2().read();
        star1 = i2c.star1().read();

        if star1.addr() {
            break;
        }
    }

    if star2.tra() {
        i2c.datar().write(|r| {
            r.set_datar(0x42);
        });
        // Read star2 again, to clear ADDR and start data transfer
        i2c.star2().read();
        i2c.ctlr1().modify(|r| {
            r.set_stop(true);
        });
        writeln!(uart, "Written\r").unwrap();
    } else {
        let mut a = [0 as u8; 8];
        let mut i = 0;
        let mut status = i2c.star1().read();
        while !status.stopf() {
            if status.btf() {
                if i < 8 {
                    a[i] = i2c.datar().read().datar();
                }
                i += 1;
            }
            status = i2c.star1().read();
        }
        if i2c.star1().read().btf() {
            if i < 8 {
                a[i] = i2c.datar().read().datar();
            }
            i += 1;
        }
        i2c.ctlr1().modify(|r| {
            r.set_ack(false);
        });
        writeln!(uart, "Received {}, {:?}\r", i, a).unwrap();
    }
}
