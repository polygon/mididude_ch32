#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use ch32_hal::pac::gpio::vals::{Cnf, Mode};
use core::fmt::Write;
use hal::delay::Delay;
use hal::gpio::{Level, Output};
use hal::i2c::I2c;
use hal::println;
use hal::usart::UartTx;
use numtoa::NumToA;
use {ch32_hal as hal, panic_halt as _};

use mididude_adc::i2c_device::{self, init_i2c_device, monitor_addr};

#[qingke_rt::entry]
fn main() -> ! {
    //hal::debug::SDIPrint::enable();
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSI;
    let p = hal::init(config);

    let mut led = Output::new(p.PD6, Level::Low, Default::default());

    let mut uart = UartTx::new_blocking(p.USART1, p.PC0, Default::default()).unwrap();

    hal::pac::GPIOC.cfglr().modify(|r| {
        r.set_mode(1, Mode::OUTPUT_50MHZ);
        r.set_mode(2, Mode::OUTPUT_50MHZ);
        r.set_cnf(1, Cnf::AF_OPEN_DRAIN_OUT);
        r.set_cnf(2, Cnf::AF_OPEN_DRAIN_OUT);
    });

    let mut adc = hal::adc::Adc::new(p.ADC1, Default::default());

    let mut pin1 = p.PA2;
    let mut pin2 = p.PA1;

    writeln!(uart, "Formatted {}\n", 12).unwrap();

    init_i2c_device(0x10, &mut uart);
    loop {
        monitor_addr(&mut uart);
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
