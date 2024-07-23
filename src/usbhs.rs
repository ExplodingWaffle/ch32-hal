//! UsbHs High speed USB peripheral

use core::arch::asm;
use core::cell::UnsafeCell;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::ptr::addr_of_mut;
use core::sync::atomic::{AtomicBool, AtomicU16, Ordering};
use core::task::{Context, Poll};

use embassy_sync::waitqueue::AtomicWaker;
use embassy_usb_driver::{
    Bus as _, Direction, EndpointAddress, EndpointAllocError, EndpointError, EndpointIn, EndpointInfo, EndpointOut,
    EndpointType, Event, Unsupported,
};
use qingke::riscv::asm::delay;

use crate::internal::drop::OnDrop;
use crate::interrupt::typelevel::Interrupt;
use crate::{interrupt, into_ref, pac, peripherals, println, Peripheral, PeripheralRef};

#[derive(Default)]
pub struct Config {}

pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

struct EpOutState {
    recv_len: AtomicU16,
    waker: AtomicWaker,
}

impl EpOutState {
    const fn new() -> Self {
        Self {
            recv_len: AtomicU16::new(u16::MAX),
            waker: AtomicWaker::new(),
        }
    }
}

struct EpInState {
    acked: AtomicBool,
    waker: AtomicWaker,
}

impl EpInState {
    const fn new() -> Self {
        Self {
            acked: AtomicBool::new(false),
            waker: AtomicWaker::new(),
        }
    }
}

static EP_OUT_STATE: [EpOutState; 16] = [const { EpOutState::new() }; 16];
static EP_IN_STATE: [EpInState; 16] = [const { EpInState::new() }; 16];

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let r = T::regs();

        let flag = r.int_fg().read();
        // println!("irq flag: {:#010b}", flag.0);

        // let en = r.int_en().read();
        // println!("en:  {:#010b}", en.0);

        // interrupt number is 85
        // let bit = 1 << (85 - 64);
        // println!("bit:   {:#034b}", bit);
        // println!("isr:   {:#034b}", pac::PFIC.isr3().read().0);
        // println!("ipr:   {:#034b}", pac::PFIC.ipr3().read().0);
        // println!(
        //     "iactr: {:#034b}",
        //     core::ptr::read_volatile(pac::PFIC.iactr3().as_ptr()).0
        // );

        if flag.bus_rst() {
            //mask the interrupt and let the main thread handle it
            r.int_en().modify(|w| w.set_bus_rst(false));
            BUS_WAKER.wake();
        };
        if flag.suspend() {
            //mask the interrupt and let the main thread handle it
            // println!("misc:suspend: {}", r.mis_st().read().suspend());
            r.int_en().modify(|w| w.set_suspend(false));
            // println!("int_en: {:#010b}", r.int_en().read().0);
            // println!("flag:   {:#010b}", r.int_fg().read().0);
            BUS_WAKER.wake();
            // println!("waker woken");
        };
        if flag.setup_act() {
            //mask the interrupt and let the main thread handle it
            r.int_en().modify(|w| w.set_setup_act(false));
            CONTROL_WAKER.wake();
        };
        if flag.transfer() {
            let d = T::dregs();

            let status = r.int_st().read();
            println!("irq stat: {:#010b}", status.0);

            // let pid = d.ep_rx_ctrl(0).read().mask_uep_r_res();
            // let tog = d.ep_rx_ctrl(0).read().mask_uep_r_tog();
            // println!("ep0 pid: {:#04b}", pid);
            // println!("ep0 tog: {:#04b}", tog);

            if !status.is_nak() {
                match status.token() {
                    // OUT
                    0b00 => {
                        if status.tog_ok() {
                            d.ep_rx_ctrl(0).modify(|w| {
                                // NAK
                                w.set_mask_uep_r_res(0b10);
                                // data tog
                                w.set_mask_uep_r_tog(w.mask_uep_r_tog() ^ 0b01);
                            });
                            let state = &EP_OUT_STATE[status.endp() as usize];
                            state.recv_len.store(d.rx_len().read(), Ordering::Relaxed);
                            state.waker.wake();
                        }
                    }
                    // SOF
                    0b01 => {}
                    // IN
                    0b10 => {
                        d.ep_tx_ctrl(0).modify(|w| {
                            // NAK
                            w.set_mask_uep_t_res(0b10);
                            // data tog
                            w.set_mask_uep_t_tog(w.mask_uep_t_tog() ^ 0b01);
                        });
                        let state = &EP_IN_STATE[status.endp() as usize];
                        state.acked.store(true, Ordering::Relaxed);
                        state.waker.wake();
                    }
                    // SETUP
                    0b11 => {}
                    _ => unreachable!(),
                }
            } else {
                // NAK
                r.int_fg().write(|w| w.set_is_nak(true));
            }

            r.int_fg().write(|w| w.set_transfer(true));
        }
    }
}

pub struct WakeupInterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::WakeupInterrupt> for WakeupInterruptHandler<T> {
    unsafe fn on_interrupt() {
        //todo!()
    }
}

pub struct UsbHs<'d, T: Instance> {
    _peri: PeripheralRef<'d, T>,
}

impl<'d, T: Instance> UsbHs<'d, T> {
    pub fn new(
        peri: impl Peripheral<P = T> + 'd,
        _irqs: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>>
            + interrupt::typelevel::Binding<T::WakeupInterrupt, WakeupInterruptHandler<T>>
            + 'd,
        dp: impl Peripheral<P = impl DpPin<T, 0> + 'd>,
        dm: impl Peripheral<P = impl DmPin<T, 0> + 'd>,
        config: Config,
    ) -> Self {
        into_ref!(peri, dp, dm);

        T::enable_and_reset();

        //T::Interrupt::unpend();
        unsafe {
            //T::Interrupt::enable();
            //T::Interrupt::pend()
        }

        // let r = T::regs();

        // r.ctrl().write(|w| {
        //     w.set_dev_pu_en(true);
        //     w.set_dma_en(true);
        //     w.set_reset_sie(false);
        //     w.set_int_busy(true)
        // });

        Self { _peri: peri }
    }
}

impl<'d, T: Instance> embassy_usb_driver::Driver<'d> for UsbHs<'d, T> {
    type EndpointOut = Endpoint<'d, T, Out>;
    type EndpointIn = Endpoint<'d, T, In>;
    type ControlPipe = ControlPipe<'d, T>;
    type Bus = Bus<'d, T>;

    fn alloc_endpoint_out(
        &mut self,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointOut, EndpointAllocError> {
        todo!()
    }

    fn alloc_endpoint_in(
        &mut self,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointIn, EndpointAllocError> {
        todo!()
    }

    fn start(mut self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        let r = T::regs();
        let d = T::dregs();
        let h = T::hregs();

        //println!("start");

        // assert!(control_max_packet_size >= 8 && control_max_packet_size <= 64);

        r.ctrl().write(|w| {
            w.set_clr_all(true);
            w.set_reset_sie(true);
        });

        //10us delay from sdk
        delay(1500);

        r.ctrl().modify(|w| {
            w.set_reset_sie(false);
        });

        h.ctrl().write(|w| {
            w.set_phy_suspendm(true);
        });

        r.ctrl().write(|w| {
            w.set_dma_en(true);
            w.set_int_busy(true);
            w.set_speed_type(0b00);
        });

        r.int_en().write(|w| {
            // w.set_bus_rst(true);
            // w.set_dev_nak(true);
            // w.set_setup_act(true);
            // w.set_suspend(true);
            w.set_transfer(true);
        });

        //sdk inits endpoints here

        // d.ep_config().write(|w| {
        //     w.set_r_en(0, true);
        //     w.set_t_en(0, true);
        // });

        d.ep0_dma().write(|w| w.0 = unsafe { addr_of_mut!(CTRL_BUF) } as u32);
        d.ep_max_len(0).write(|w| w.set_len(64));
        d.ep_rx_ctrl(0).write(|w| w.set_mask_uep_r_res(0b10));

        d.ep_t_len(0).write(|w| w.set_len(0));
        d.ep_tx_ctrl(0).write(|w| w.set_mask_uep_t_res(0b10));
        // d.ep_rx_ctrl(0).write(|w| w.set_mask_uep_r_res(0b10));

        // r.ctrl().modify(|w| {
        //     w.set_dev_pu_en(true);
        // });

        // println!("usbhs started");
        // println!("init flags: {:#010b}", r.int_fg().read().0);

        // r.int_fg().write(|w| {
        //     w.0 = 0xff;
        // });

        unsafe {
            // T::Interrupt::unpend();
            T::Interrupt::enable();
            //T::Interrupt::pend();
        }
        // println!("interrupts enabled");

        // unsafe {
        //     asm!("ebreak");
        // }
        (
            Bus {
                _phantom: PhantomData,
                powered: false,
            },
            ControlPipe {
                _phantom: PhantomData,
                max_packet_size: control_max_packet_size,
            },
        )
    }
}

impl<'d, T: Instance> Drop for UsbHs<'d, T> {
    fn drop(&mut self) {
        //todo!()
    }
}

pub struct Bus<'d, T: Instance> {
    _phantom: PhantomData<&'d mut T>,
    powered: bool,
}

static BUS_WAKER: AtomicWaker = AtomicWaker::new();

impl<'d, T: Instance> embassy_usb_driver::Bus for Bus<'d, T> {
    async fn enable(&mut self) {
        println!("bus enable");
        let r = T::regs();
        r.ctrl().modify(|w| {
            w.set_dev_pu_en(true);
        });
        // nop for now
        // enable comes after powered, so we should be enabling the
        // pull-ups here and possibly other stuff
    }

    async fn disable(&mut self) {
        println!("bus disable");
        let r = T::regs();
        r.ctrl().modify(|w| {
            w.set_dev_pu_en(false);
        });
        // nop
    }

    async fn poll(&mut self) -> Event {
        let r = T::regs();

        //disable interrupts on drop
        let on_drop = OnDrop::new(|| {
            r.int_en().modify(|w| {
                w.set_bus_rst(false);
                w.set_suspend(false);
            });
            // r.int_fg().write(|w| {
            //     w.set_bus_rst(true);
            //     w.set_suspend(true);
            // });
            // println!("poll drop");
        });
        let e = poll_fn(|cx| {
            BUS_WAKER.register(cx.waker());

            //TODO: real vbus detection
            if !self.powered {
                self.powered = true;
                return Poll::Ready(Event::PowerDetected);
            }

            r.int_en().modify(|w| {
                w.set_bus_rst(true);
                w.set_suspend(true);
            });

            let en = r.int_en().read();
            // println!("poll en: {:#010b}", en.0);

            let flag = r.int_fg().read();
            // println!("poll fg: {:#010b}", flag.0);

            if flag.bus_rst() {
                // println!("poll reset");
                let mis_st = r.mis_st().read();
                println!("reset: {}", mis_st.bus_reset());
                r.int_fg().write(|w| w.set_bus_rst(true));
                if mis_st.bus_reset() {
                    return Poll::Ready(Event::Reset);
                }
            }

            if flag.suspend() {
                // println!("poll suspend");
                r.int_fg().write(|w| w.set_suspend(true));
                let mis_st = r.mis_st().read();
                println!("suspend: {}", mis_st.suspend());
                if mis_st.suspend() {
                    return Poll::Ready(Event::Suspend);
                } else {
                    return Poll::Ready(Event::Resume);
                }
            }

            Poll::Pending
        })
        .await;
        drop(on_drop);
        // let en = r.int_en().read();
        // println!("end en: {:#010b}", en.0);

        // let flag = r.int_fg().read();
        // println!("end fg: {:#010b}", flag.0);
        e
    }

    fn endpoint_set_enabled(&mut self, ep_addr: EndpointAddress, enabled: bool) {
        println!("endpoint set enabled");
        todo!()
    }

    fn endpoint_set_stalled(&mut self, ep_addr: EndpointAddress, stalled: bool) {
        println!("endpoint set stalled");
        todo!()
    }

    fn endpoint_is_stalled(&mut self, ep_addr: EndpointAddress) -> bool {
        println!("endpoint is stalled");
        todo!()
    }

    async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
        println!("remote wakeup");
        todo!()
    }
}

trait Dir {
    fn dir() -> Direction;
}

/// Marker type for the "IN" direction.
pub enum In {}
impl Dir for In {
    fn dir() -> Direction {
        Direction::In
    }
}

/// Marker type for the "OUT" direction.
pub enum Out {}
impl Dir for Out {
    fn dir() -> Direction {
        Direction::Out
    }
}

/// USB endpoint.
pub struct Endpoint<'d, T: Instance, D> {
    _phantom: PhantomData<(&'d mut T, D)>,
    info: EndpointInfo,
}

impl<'d, T: Instance, D> embassy_usb_driver::Endpoint for Endpoint<'d, T, D> {
    fn info(&self) -> &EndpointInfo {
        todo!()
    }

    async fn wait_enabled(&mut self) {
        todo!()
    }
}

impl<'d, T: Instance> EndpointOut for Endpoint<'d, T, Out> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        todo!()
    }
}

impl<'d, T: Instance> EndpointIn for Endpoint<'d, T, In> {
    async fn write(&mut self, buf: &[u8]) -> Result<(), EndpointError> {
        todo!()
    }
}

pub struct ControlPipe<'d, T: Instance> {
    _phantom: PhantomData<&'d mut T>,
    max_packet_size: u16,
}

static CONTROL_WAKER: AtomicWaker = AtomicWaker::new();

#[repr(align(4))]
struct CtrlRam([u8; 64]);

#[repr(align(4))]
struct SetupRam([u8; 8]);

static mut CTRL_BUF: CtrlRam = CtrlRam([0x69; 64]);

impl<'d, T: Instance> embassy_usb_driver::ControlPipe for ControlPipe<'d, T> {
    fn max_packet_size(&self) -> usize {
        self.max_packet_size as usize
    }

    async fn setup(&mut self) -> [u8; 8] {
        // println!("setup");

        let r = T::regs();
        let d = T::dregs();

        let on_drop = OnDrop::new(|| {
            r.int_en().modify(|w| {
                w.set_setup_act(false);
            });
            // r.int_fg().write(|w| {
            //     w.set_setup_act(true);
            // });
            // println!("setup drop");
        });

        //control is implicitly always enabled?

        let ret = poll_fn(|cx| {
            CONTROL_WAKER.register(cx.waker());

            r.int_en().modify(|w| {
                w.set_setup_act(true);
            });

            let en = r.int_en().read();
            // println!("stup en: {:#010b}", en.0);

            let flag = r.int_fg().read();
            // println!("stup fg: {:#010b}", flag.0);

            if flag.setup_act() {
                r.int_fg().write(|w| w.set_setup_act(true));

                // NAK
                d.ep_rx_ctrl(0).write(|w| w.set_mask_uep_r_res(0b10));
                d.ep_tx_ctrl(0).write(|w| w.set_mask_uep_t_res(0b10));

                let data: [u8; 8] = unsafe { *&CTRL_BUF.0[0..8].try_into().unwrap() };
                //let data = unsafe { *addr_of_mut!(SETUP_BUF) }.0;
                println!("setup data: {:x?}", data);
                return Poll::Ready(data);
            }
            Poll::Pending
        })
        .await;
        drop(on_drop);
        ret
    }

    async fn data_out(&mut self, buf: &mut [u8], first: bool, last: bool) -> Result<usize, EndpointError> {
        println!("data out");
        // todo!()
        Ok(0)
    }

    async fn data_in(&mut self, data: &[u8], first: bool, last: bool) -> Result<(), EndpointError> {
        println!("data in");
        // println!("data: {:x?}", data);
        //transmit data
        if data.len() > 64 {
            return Err(EndpointError::BufferOverflow);
        }

        let buf = unsafe { &mut CTRL_BUF.0[..data.len()] };
        buf.copy_from_slice(data);

        // println!("buf: {:x?}", buf);

        let d = T::dregs();
        let r = T::regs();

        EP_IN_STATE[0].acked.store(false, Ordering::Relaxed);

        d.ep_t_len(0).write(|w| w.set_len(data.len() as u16));

        // d.ep_tx_ctrl(0).write(|w| {
        //     // data stage starts with DATA1
        //     w.set_mask_uep_t_tog(0b01);
        //     w.set_mask_uep_t_res(0b10);
        // });

        // while !r.mis_st().read().sie_free() {
        //     println!("sie not free");
        // }

        d.ep_tx_ctrl(0).write(|w| {
            // data stage starts with DATA1
            w.set_mask_uep_t_tog(0b01);
            w.set_mask_uep_t_res(0b00);
        });

        poll_fn(|cx| {
            EP_IN_STATE[0].waker.register(cx.waker());

            println!("poll data in");

            if EP_IN_STATE[0].acked.load(Ordering::Relaxed) {
                return Poll::Ready(());
            }

            Poll::Pending
        })
        .await;

        println!("data in done");

        // we need to do a data OUT (recv) for the status stage

        if last {
            println!("status stage");
            // status stage = recieve, DATA1, ack
            d.ep_rx_ctrl(0).write(|w| {
                w.set_mask_uep_r_res(0b00);
                w.set_mask_uep_r_tog(0b01)
            });

            EP_OUT_STATE[0].recv_len.store(u16::MAX, Ordering::Relaxed);

            poll_fn(|cx| {
                // println!("poll status");
                EP_OUT_STATE[0].waker.register(cx.waker());

                if EP_OUT_STATE[0].recv_len.load(Ordering::Relaxed) != u16::MAX {
                    return Poll::Ready(());
                }

                Poll::Pending
            })
            .await;
        }

        Ok(())
    }

    async fn accept(&mut self) {
        println!("accept");

        let d = T::dregs();

        EP_IN_STATE[0].acked.store(false, Ordering::Relaxed);

        d.ep_t_len(0).write(|w| w.set_len(0));

        d.ep_tx_ctrl(0).write(|w| {
            // status stage starts with DATA1
            w.set_mask_uep_t_tog(0b01);
            w.set_mask_uep_t_res(0b00);
        });

        poll_fn(|cx| {
            EP_IN_STATE[0].waker.register(cx.waker());

            if EP_IN_STATE[0].acked.load(Ordering::Relaxed) {
                return Poll::Ready(());
            }

            Poll::Pending
        })
        .await;
    }

    async fn reject(&mut self) {
        println!("reject");
        let d = T::dregs();

        // EP_IN_STATE[0].acked.store(false, Ordering::Relaxed);

        d.ep_t_len(0).write(|w| w.set_len(0));

        d.ep_tx_ctrl(0).write(|w| {
            // status stage starts with DATA1
            // w.set_mask_uep_t_tog(0b01);
            w.set_mask_uep_t_res(0b11);
        });
    }

    async fn accept_set_address(&mut self, addr: u8) {
        println!("accept set address");
        let r = T::regs();
        self.accept().await;
        r.dev_ad().write(|w| w.set_addr(addr));
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

trait SealedInstance: crate::peripheral::RccPeripheral {
    fn regs() -> crate::pac::usb::Usb;
    fn dregs() -> crate::pac::usb::Usbd;
    fn hregs() -> crate::pac::usb::Usbh;
    fn state() -> &'static State;
}

/// UsbHs peripheral instance
#[allow(private_bounds)]
pub trait Instance: SealedInstance + 'static {
    /// Regular interrupt for this instance
    type Interrupt: interrupt::typelevel::Interrupt;
    /// Wakeup interrupt for this instance
    type WakeupInterrupt: interrupt::typelevel::Interrupt;
}

foreach_peripheral!(
    (usb, $inst:ident) => {
        impl SealedInstance for peripherals::$inst {
            fn regs() -> crate::pac::usb::Usb {
                crate::pac::$inst
            }

            fn dregs() -> crate::pac::usb::Usbd {
                unsafe {
                    core::mem::transmute(crate::pac::$inst)
                }
            }

            fn hregs() -> crate::pac::usb::Usbh {
                unsafe {
                    core::mem::transmute(crate::pac::$inst)
                }
            }

            fn state() -> &'static State {
                static STATE: State = State::new();
                &STATE
            }
        }

        impl Instance for peripherals::$inst {
            type Interrupt = crate::_generated::peripheral_interrupts::$inst::GLOBAL;
            type WakeupInterrupt = crate::_generated::peripheral_interrupts::$inst::WAKEUP;
        }
    };
);

pin_trait!(DmPin, Instance);
pin_trait!(DpPin, Instance);
