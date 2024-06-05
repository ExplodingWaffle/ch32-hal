#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hal::adc::SampleTime;
use hal::gpio::{AnyPin, Level, Output, Pin};
use {ch32_hal as hal, panic_halt as _};

#[embassy_executor::task(pool_size = 3)]
async fn blink(pin: AnyPin, interval_ms: u64) {
    let mut led = Output::new(pin, Level::Low, Default::default());

    loop {
        led.set_high();
        Timer::after(Duration::from_millis(interval_ms)).await;
        led.set_low();
        Timer::after(Duration::from_millis(interval_ms)).await;
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    let p = hal::init(Default::default());
    hal::embassy::init();
    info!("Hello, World!");

    let mut adc = hal::adc::Adc::new(p.ADC1, Default::default());

    let mut ch = p.PA0;

    // GPIO
    spawner.spawn(blink(p.PA15.degrade(), 1000)).unwrap();
    spawner.spawn(blink(p.PB4.degrade(), 100)).unwrap();
    // spawner.spawn(blink(p.PB8.degrade(), 100)).unwrap();
    loop {
        println!("Starting conversion!");
        let val = adc.convert(&mut ch, SampleTime::CYCLES1_5);

        println!("val => {}", val);
        Timer::after_millis(100).await;
    }
}
