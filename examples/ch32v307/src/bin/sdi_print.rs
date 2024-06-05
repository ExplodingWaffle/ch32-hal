#![no_std]
#![no_main]

//! SDI debug print

//use hal::println;
use defmt::*;
use hal::gpio::{Level, Output};
use {ch32_hal as hal, panic_halt as _};

#[qingke_rt::entry]
fn main() -> ! {
    let p = hal::init(Default::default());
    hal::embassy::init();

    let mut led = Output::new(p.PA15, Level::High, Default::default());

    led.toggle();
    qingke::riscv::asm::delay(10_000_000);
    led.toggle();
    qingke::riscv::asm::delay(10_000_000);
    led.toggle();
    qingke::riscv::asm::delay(10_000_000);

    hal::debug::SDIPrint::enable();

    println!("hello world!");

    //println!("Flash size: {}kb", hal::signature::flash_size_kb());
    println!("Chip UID: {=[u8]:x}", hal::signature::unique_id());
    let chip_id = hal::signature::chip_id();
    println!("Chip {}, DevID: 0x{:x}", chip_id.name(), chip_id.dev_id());

    loop {
        println!("hello world!");

        unsafe {
            qingke::riscv::asm::delay(10_000_000);
        }
    }
}
