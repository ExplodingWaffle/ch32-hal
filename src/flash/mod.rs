use core::marker::PhantomData;

#[cfg(any(ch32v2, ch32v3, ch32f2))]
#[path = "v3.rs"]
mod flash_impl;

use embassy_hal_internal::{into_ref, Peripheral, PeripheralRef};
use embedded_storage::nor_flash::NorFlashErrorKind;
use flash_impl::{ERASE_SIZE, FLASH_BASE, FLASH_SIZE, READ_SIZE, WRITE_SIZE};

/// Blocking flash mode typestate.
pub enum Blocking {}
/// Async flash mode typestate.
pub enum Async {}

/// Internal flash memory driver.
pub struct Flash<'d, MODE = Async> {
    pub(crate) _inner: PeripheralRef<'d, crate::peripherals::FLASH>,
    pub(crate) _mode: PhantomData<MODE>,
}

impl<'d> Flash<'d, Blocking> {
    /// Create a new flash driver, usable in blocking mode.
    pub fn new_blocking(p: impl Peripheral<P = crate::peripherals::FLASH> + 'd) -> Self {
        into_ref!(p);

        Self {
            _inner: p,
            _mode: PhantomData,
        }
    }
}

impl<'d, MODE> Flash<'d, MODE> {
    /// Blocking read.
    ///
    /// NOTE: `offset` is an offset from the flash start, NOT an absolute address.
    /// For example, to read address `0x0800_1234` you have to use offset `0x1234`.
    pub fn blocking_read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Error> {
        flash_impl::blocking_read(offset, bytes)
    }

    /// Blocking write.
    ///
    /// NOTE: `offset` is an offset from the flash start, NOT an absolute address.
    /// For example, to write address `0x0800_1234` you have to use offset `0x1234`.
    pub fn blocking_write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Error> {
        unsafe { flash_impl::blocking_write(offset, bytes) }
    }

    /// Blocking erase.
    ///
    /// NOTE: `from` and `to` are offsets from the flash start, NOT an absolute address.
    /// For example, to erase address `0x0801_0000` you have to use offset `0x1_0000`.
    pub fn blocking_erase(&mut self, from: u32, to: u32) -> Result<(), Error> {
        unsafe { flash_impl::blocking_erase(from, to) }
    }
}

/// Flash error
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    Protected,
    Unaligned,
    Size,
}

impl embedded_storage::nor_flash::NorFlashError for Error {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            Error::Size => NorFlashErrorKind::OutOfBounds,
            Error::Unaligned => NorFlashErrorKind::NotAligned,
            _ => NorFlashErrorKind::Other,
        }
    }
}

impl<MODE> embedded_storage::nor_flash::ErrorType for Flash<'_, MODE> {
    type Error = Error;
}

impl<MODE> embedded_storage::nor_flash::ReadNorFlash for Flash<'_, MODE> {
    const READ_SIZE: usize = READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_read(offset, bytes)
    }

    fn capacity(&self) -> usize {
        FLASH_SIZE
    }
}

impl<MODE> embedded_storage::nor_flash::NorFlash for Flash<'_, MODE> {
    const WRITE_SIZE: usize = WRITE_SIZE;
    const ERASE_SIZE: usize = ERASE_SIZE;
    const ERASE_VALUE: &'static [u8] = &[0x39, 0xe3];

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        self.blocking_write(offset, bytes)
    }

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        self.blocking_erase(from, to)
    }
}
