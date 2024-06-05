#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::arch::asm;

use defmt::{panic, *};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use futures::future::join;
use hal::gpio::{AnyPin, Level, Output, Pin};
use hal::usbhd::{self, Instance, Usbhd};
use hal::{bind_interrupts, peripherals, rcc};
use {ch32_hal as hal, panic_halt as _};

bind_interrupts!(struct Irqs {
    USBHS => usbhd::InterruptHandler<peripherals::USBHD>;
    USBHSWAKEUP => usbhd::WakeupInterruptHandler<peripherals::USBHD>;
});

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) {
    let mut c = hal::Config::default();
    c.rcc = rcc::Config::SYSCLK_FREQ_144MHZ_HSE;
    let p = hal::init(c);
    hal::embassy::init();

    //info!("Hello, World!");

    spawner.spawn(blink(p.PA15.degrade(), 5000)).unwrap();
    unsafe {
        asm!("ebreak");
    }

    info!("Hello World!");

    let mut config = usbhd::Config::default();

    let driver = Usbhd::new(p.USBHD, Irqs, p.PB7, p.PB6, config);

    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB-Serial Example");
    config.serial_number = Some("123456");

    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);

    let mut usb = builder.build();

    let usb_fut = usb.run();

    let echo_fut = async {
        loop {
            class.wait_connection().await;
            info!("Connected");
            let _ = echo(&mut class).await;
            info!("Disconnected");
        }
    };

    join(usb_fut, echo_fut).await;
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

async fn echo<'d, T: Instance + 'd>(class: &mut CdcAcmClass<'d, Usbhd<'d, T>>) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = class.read_packet(&mut buf).await?;
        let data = &buf[..n];
        info!("data: {:x}", data);
        class.write_packet(data).await?;
    }
}

#[embassy_executor::task]
async fn blink(pin: AnyPin, interval_ms: u64) {
    let mut led = Output::new(pin, Level::Low, Default::default());
    loop {
        led.set_high();
        Timer::after(Duration::from_millis(interval_ms)).await;
        led.set_low();
        Timer::after(Duration::from_millis(interval_ms)).await;
    }
}
