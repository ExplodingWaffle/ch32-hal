use core::ptr::{slice_from_raw_parts, write_volatile};

use super::Error;
use crate::pac::flash::regs::*;
use crate::pac::FLASH;
use crate::println;

pub(crate) const FLASH_BASE: usize = 0x0800_0000;
pub(crate) const FLASH_SIZE: usize = 480 * 1024;
pub(crate) const READ_SIZE: usize = 1;
pub(crate) const WRITE_SIZE: usize = 2;
pub(crate) const ERASE_SIZE: usize = 4 * 1024;

unsafe fn blocking_wait_ready() -> Result<(), Error> {
    loop {
        let sr = FLASH.statr().read();

        if !sr.bsy() {
            if sr.wrprterr() {
                return Err(Error::Protected);
            }

            FLASH.statr().write(|w| w.set_eop(true));
            return Ok(());
        }
    }
}

unsafe fn unlock() {
    FLASH.keyr().write_value(Keyr(0x45670123));
    FLASH.keyr().write_value(Keyr(0xCDEF89AB));
    while FLASH.ctlr().read().lock() {}
}

unsafe fn lock() {
    FLASH.ctlr().modify(|w| w.set_lock(true));
}

pub(crate) fn blocking_read(offset: u32, bytes: &mut [u8]) -> Result<(), Error> {
    if offset as usize + bytes.len() > FLASH_SIZE {
        return Err(Error::Size);
    }

    let flash_data = unsafe { core::slice::from_raw_parts((FLASH_BASE + offset as usize) as *const u8, bytes.len()) };
    bytes.copy_from_slice(flash_data);
    Ok(())
}

pub(crate) unsafe fn blocking_write(offset: u32, bytes: &[u8]) -> Result<(), Error> {
    if offset as usize + bytes.len() > FLASH_SIZE {
        return Err(Error::Size);
    }
    if offset as usize % WRITE_SIZE != 0 || bytes.len() % WRITE_SIZE != 0 {
        return Err(Error::Unaligned);
    }

    let mut addr = FLASH_BASE + offset as usize;

    unlock();
    FLASH.ctlr().modify(|w| w.set_pg(true));

    for chunk in bytes.chunks_exact(WRITE_SIZE) {
        let word: u16 = u16::from_ne_bytes(chunk.try_into().unwrap());
        write_volatile(addr as *mut u16, word);

        blocking_wait_ready()?;

        addr += WRITE_SIZE;
    }

    FLASH.ctlr().modify(|w| w.set_pg(false));
    lock();
    Ok(())
}

pub(crate) unsafe fn blocking_erase(from: u32, to: u32) -> Result<(), Error> {
    if from as usize % ERASE_SIZE != 0 || to as usize % ERASE_SIZE != 0 {
        return Err(Error::Unaligned);
    }
    if from > to {
        return Err(Error::Size);
    }

    unlock();
    FLASH.ctlr().modify(|w| w.set_per(true));

    let mut addr = FLASH_BASE as u32 + from;

    while addr < FLASH_BASE as u32 + to {
        FLASH.addr().write_value(Addr(addr));
        FLASH.ctlr().modify(|w| w.set_strt(true));
        blocking_wait_ready()?;
        addr += ERASE_SIZE as u32;
    }

    FLASH.ctlr().modify(|w| w.set_per(false));
    lock();

    Ok(())
}
