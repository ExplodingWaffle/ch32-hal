use core::cell::{Cell, RefCell, UnsafeCell};
use core::sync::atomic::{AtomicBool, Ordering};

use critical_section::{Mutex, RestoreState};
use defmt::Encoder;

use crate::debug::{regs, SDIPrint};

static mut ENCODER: Encoder = Encoder::new();
static TAKEN: AtomicBool = AtomicBool::new(false);
static mut RESTORE: RestoreState = RestoreState::invalid();

#[defmt::global_logger]
struct Logger;

unsafe impl defmt::Logger for Logger {
    fn acquire() {
        let restore = unsafe { critical_section::acquire() };

        if TAKEN.load(Ordering::Relaxed) {
            panic!("defmt logger taken reentrantly")
        }
        TAKEN.store(true, Ordering::Relaxed);

        unsafe {
            RESTORE = restore;
        }

        unsafe {
            ENCODER.start_frame(do_write);
        }
    }

    unsafe fn flush() {
        //nop
    }

    unsafe fn release() {
        unsafe {
            ENCODER.end_frame(do_write);
        }

        TAKEN.store(false, Ordering::Relaxed);

        let restore = RESTORE;

        unsafe {
            critical_section::release(restore);
        }
    }

    unsafe fn write(bytes: &[u8]) {
        unsafe {
            ENCODER.write(bytes, do_write);
        }
    }
}

fn do_write(bytes: &[u8]) {
    let mut data = [0u8; 8];
    for chunk in bytes.chunks(7) {
        data[1..chunk.len() + 1].copy_from_slice(chunk);
        data[0] = chunk.len() as u8;

        // data1 is the last 4 bytes of data
        let data1 = u32::from_le_bytes(data[4..].try_into().unwrap());
        let data0 = u32::from_le_bytes(data[..4].try_into().unwrap());

        while SDIPrint::is_busy() {}

        unsafe {
            core::ptr::write_volatile(regs::DEBUG_DATA1_ADDRESS, data1);
            core::ptr::write_volatile(regs::DEBUG_DATA0_ADDRESS, data0);
        }
    }
}
