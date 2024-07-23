#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]
#![feature(naked_functions)]
use core::arch::asm;

use ch32_hal as hal;
//use defmt::{panic, *};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use hal::gpio::{AnyPin, Level, Output, Pin};
use hal::usbhs::{self, Instance, UsbHs};
use hal::{bind_interrupts, peripherals, println, rcc};
use qingke::riscv::asm; //, panic_halt as _};

bind_interrupts!(struct Irqs {
    USBHS => usbhs::InterruptHandler<peripherals::USBHS>;
    USBHS_WKUP => usbhs::WakeupInterruptHandler<peripherals::USBHS>;
});

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) {
    let mut c = hal::Config::default();
    c.rcc = rcc::Config::SYSCLK_FREQ_144MHZ_HSE;
    let p = hal::init(c);
    hal::embassy::init();

    Timer::after(Duration::from_millis(50)).await;

    hal::debug::SDIPrint::enable();

    Timer::after(Duration::from_millis(50)).await;

    println!("");
    println!("hello world!");

    // unsafe {
    //     asm!("ebreak");
    // }

    spawner.spawn(blink(p.PA10.degrade(), 500)).unwrap();

    // Timer::after(Duration::from_millis(5000)).await;

    let config = usbhs::Config::default();

    let driver = UsbHs::new(p.USBHS, Irqs, p.PB7, p.PB6, config);

    // println!("USBHS initialized");

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

    // println!("config initialized");

    let mut state = State::new();

    let mut handler = MyHandler {};

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    // println!("builder initialized");

    // let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);

    // println!("class initialized");

    builder.handler(&mut handler);

    let mut usb = builder.build();

    println!("usb initialized");

    let usb_fut = usb.run();

    // println!("usb_fut initialized");

    // let echo_fut = async {
    //     loop {
    //         class.wait_connection().await;
    //         //info!("Connected");
    //         let _ = echo(&mut class).await;
    //         //info!("Disconnected");
    //     }
    // };

    // join(usb_fut, echo_fut).await;
    usb_fut.await;
}

struct MyHandler {}

impl embassy_usb::Handler for MyHandler {
    fn enabled(&mut self, _enabled: bool) {
        println!("enabled: {}", _enabled);
    }

    fn reset(&mut self) {
        println!("reset");
    }

    fn addressed(&mut self, _addr: u8) {
        println!("addressed: {}", _addr);
    }

    fn configured(&mut self, _configured: bool) {
        println!("configured: {}", _configured);
    }

    fn suspended(&mut self, _suspended: bool) {
        println!("suspended: {}", _suspended);
    }

    fn remote_wakeup_enabled(&mut self, _enabled: bool) {
        println!("remote_wakeup_enabled: {}", _enabled);
    }

    fn set_alternate_setting(&mut self, iface: embassy_usb::types::InterfaceNumber, alternate_setting: u8) {
        let _ = iface;
        let _ = alternate_setting;
        println!("set_alternate_setting: {} {}", iface.0, alternate_setting);
    }

    fn control_out(
        &mut self,
        req: embassy_usb::control::Request,
        data: &[u8],
    ) -> Option<embassy_usb::control::OutResponse> {
        let _ = (req, data);
        println!("control_out");
        None
    }

    fn control_in<'a>(
        &'a mut self,
        req: embassy_usb::control::Request,
        buf: &'a mut [u8],
    ) -> Option<embassy_usb::control::InResponse<'a>> {
        let _ = (req, buf);
        println!("control_in");
        None
    }

    fn get_string(&mut self, index: embassy_usb::types::StringIndex, lang_id: u16) -> Option<&str> {
        let _ = (index, lang_id);
        println!("get_string");
        None
    }
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

async fn echo<'d, T: Instance + 'd>(class: &mut CdcAcmClass<'d, UsbHs<'d, T>>) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = class.read_packet(&mut buf).await?;
        let data = &buf[..n];
        //info!("data: {:x}", data);
        class.write_packet(data).await?;
    }
}

#[embassy_executor::task]
async fn blink(pin: AnyPin, interval_ms: u64) {
    let mut led = Output::new(pin, Level::Low, Default::default());
    loop {
        led.toggle();
        // unsafe {
        //     let usb = hal::peripherals::USBHS::steal();
        //     use ch32_hal::interrupt::typelevel::Interrupt;
        //     <ch32_hal::peripherals::USBHS as ch32_hal::usbhs::Instance>::Interrupt::pend();
        // };
        // println!("LED toggled");
        // usbdump();
        Timer::after(Duration::from_millis(interval_ms)).await;
    }
}

fn usbdump() {
    // critical_section::with(|_| {
    //     while true {
    //         unsafe {
    //             asm!("nop");
    //         }
    //     }
    //     ()
    // });
    // use hal::pac::usb::*;
    // let r = hal::pac::USBHS;
    // let d = unsafe { Usbd::from_ptr(r.as_ptr()) };
    // let h = unsafe { Usbh::from_ptr(r.as_ptr()) };
    // println!("usb ctrl:   {:#010b}", r.ctrl().read().0);
    // println!("usb hctrl:  {:#010b}", h.ctrl().read().0);
    // println!("usb int_en: {:#010b}", r.int_en().read().0);
    // println!("usb addr:   {:#010b}", r.dev_ad().read().0);
    // println!("usb frame:  {:#018b}", r.frame_no().read().0);
    // println!("usb susp:   {:#010b}", r.suspend().read().0);
    // println!("usb speed:  {:#010b}", r.speed_type().read().0);
    // println!("usb flag:   {:#010b}", r.int_fg().read().0);
    // println!("usb misc:   {:#010b}", h.mis_st().read().0);
    // println!("usb status: {:#010b}", r.int_st().read().0);
    // println!("usb rxlen:  {:#018b}", r.rx_len().read());
    // println!("usb epcfg:  {:#034b}", d.ep_config().read().0);
    // println!("usb eptype: {:#034b}", d.ep_type().read().0);
    // println!("usb bmod:   {:#034b}", d.ep_buf_mod().read().0);
    // println!("ep0 txctrl: {:#010b}", d.ep_tx_ctrl(0).read().0);
    // println!("ep0 rxctrl: {:#010b}", d.ep_rx_ctrl(0).read().0);

    // //interrupt number is 85
    // let bit = 1 << (85 - 64);
    // println!("bit:   {:#034b}", bit);
    // println!("isr:   {:#034b}", hal::pac::PFIC.isr3().read().0);
    // println!("ipr:   {:#034b}", hal::pac::PFIC.ipr3().read().0);
    // println!("iactr: {:#034b}", unsafe {
    //     core::ptr::read_volatile(hal::pac::PFIC.iactr3().as_ptr()).0
    // });

    // unsafe {
    //     let usb = hal::peripherals::USBHS::steal();
    //     use ch32_hal::interrupt::typelevel::Interrupt;
    //     <ch32_hal::peripherals::USBHS as ch32_hal::usbhs::Instance>::Interrupt::pend();
    // };
}

#[inline(never)]
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    println!("panic");
    println!("info: {:?}", info);
    loop {
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    }
}
