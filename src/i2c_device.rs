use ch32_hal::i2c::{Instance, SclPin, SdaPin};
use ch32_hal::{pac::gpio::Gpio, Peripherals};
use core::fmt::Write;
use core::marker::PhantomData;

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
    PartialWrite(usize),
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
    pub addr: u16,
    /// Control if the peripheral should ack to and report general calls.
    pub general_call: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            addr: 0x55,
            general_call: true,
        }
    }
}

/// I2CSlave driver.
pub struct I2cSlave<'d, T: Instance> {
    phantom: PhantomData<&'d mut T>,
    config: Config,
}

impl<'d, T: Instance> I2cSlave<'d, T> {
    /// Create a new instance.
    pub fn new(
        _peri: Peri<'d, T>,
        scl: Peri<'d, impl SclPin<T>>,
        sda: Peri<'d, impl SdaPin<T>>,
        _irq: impl Binding<T::Interrupt, InterruptHandler<T>>,
        config: Config,
    ) -> Self {
        assert!(config.addr != 0);

        // Configure SCL & SDA pins
        set_up_i2c_pin(&scl);
        set_up_i2c_pin(&sda);

        let mut ret = Self {
            phantom: PhantomData,
            config,
        };

        ret.reset();

        ret
    }
}

pub fn init_i2c_pins() {
    let gpio = &ch32_hal::pac::GPIOC;
}

pub fn init_i2c_device(address: u8, uart: &mut impl Write) {
    let rcc = &ch32_hal::pac::RCC;
    rcc.apb1pcenr().modify(|r| {
        r.set_i2c1en(true);
    });
    let i2c = &ch32_hal::pac::I2C1;

    i2c.ctlr1().modify(|w| w.set_pe(false)); // disale i2c

    // soft reset
    i2c.ctlr1().modify(|w| w.set_swrst(true));
    i2c.ctlr1().modify(|w| w.set_swrst(false));

    i2c.oaddr1().modify(|r| {
        r.set_add7_1(address);
        r.set_addmode(false);
    });
    i2c.ctlr2().modify(|r| {
        r.set_freq(48);
    });
    i2c.ckcfgr().modify(|r| {
        r.set_ccr((48000000. / 200000.) as u16);
    });
    i2c.ctlr1().modify(|r| {
        r.set_pe(true);
    });
    i2c.ctlr1().modify(|r| {
        r.set_ack(true);
    });
    writeln!(uart, "I2C enabled").unwrap();
}

pub fn monitor_addr(uart: &mut impl Write) {
    let i2c = &ch32_hal::pac::I2C1;
    i2c.ctlr1().modify(|r| {
        r.set_ack(true);
    });
    while i2c.star1().read().addr() == false {}
    if i2c.star2().read().tra() {
        i2c.datar().write(|r| {
            r.set_datar(0x42);
        });
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
