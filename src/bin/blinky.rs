#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use ch32_hal as hal;
use ch32_hal::bind_interrupts;
use ch32_hal::pac::gpio::vals::{Cnf, Mode};
use core::arch::asm;
use core::fmt::Write;
use embassy_executor::{SpawnError, Spawner};
use hal::delay::Delay;
use hal::gpio::{Level, Output};
use hal::i2c::I2c;
use hal::println;
use hal::usart::UartTx;
use numtoa::NumToA;

use mididude_adc::i2c_device::{self, monitor_addr, Config, I2cSlave};

bind_interrupts!(struct Irqs {
    I2C1_EV => mididude_adc::i2c_device::EventInterruptHandler<ch32_hal::peripherals::I2C1>;
    I2C1_ER => mididude_adc::i2c_device::ErrorInterruptHandler<ch32_hal::peripherals::I2C1>;
});

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    //hal::debug::SDIPrint::enable();
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSI;
    let p = hal::init(config);

    let mut led = Output::new(p.PD6, Level::Low, Default::default());

    let mut uart = UartTx::new_blocking(p.USART1, p.PC0, Default::default()).unwrap();

    let mut config = Config::default();
    config.addr = 0x10;
    config.general_call = false;

    let mut i2c = I2cSlave::new(p.I2C1, config, p.PC1, p.PC2, Irqs);

    let mut adc = hal::adc::Adc::new(p.ADC1, Default::default());

    let mut pin1 = p.PA2;
    let mut pin2 = p.PA1;

    writeln!(uart, "Formatted {}\n", 12).unwrap();

    let mut buf = [0; 8];

    loop {
        let res = i2c.listen(&mut buf, &mut uart).await;
        match res {
            Ok(a) => writeln!(uart, "Ok: {:?}\r", a).unwrap(),
            Err(e) => writeln!(uart, "Error: {:?}\r", e).unwrap(),
        }
    }

    loop {
        led.toggle();

        let mut val1_buffer = [0u8; 20];
        let mut val2_buffer = [0u8; 20];

        let val1 = adc.convert(&mut pin1, hal::adc::SampleTime::CYCLES73);
        let val2 = adc.convert(&mut pin2, hal::adc::SampleTime::CYCLES73);

        val1.numtoa_str(10, &mut val1_buffer);
        uart.blocking_write(val1.numtoa_str(10, &mut val1_buffer).as_bytes())
            .unwrap();
        uart.blocking_write(b", ").unwrap();
        uart.blocking_write(val2.numtoa_str(10, &mut val2_buffer).as_bytes())
            .unwrap();
        uart.blocking_write(b"\r\n").unwrap();
        //hal::println!("toggle!");
        //let val = hal::pac::SYSTICK.cnt().read();
        //hal::println!("systick: {}", val);
    }
}

use core::panic::PanicInfo;

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    loop {}
}
