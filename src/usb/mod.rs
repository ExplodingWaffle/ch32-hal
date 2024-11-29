use embassy_usb_driver::EndpointAllocError;

pub(crate) struct EndpointBufferAllocator<'d> {
    // 4 aligned because we make it from EndpointDataBuffer, and make sure to
    // only slice it by multiples of 4
    buf: &'d mut [u8],
}

impl<'d> EndpointBufferAllocator<'d> {
    pub fn new<const N: usize>(ep_buffer: &'d mut EndpointBuffers<N>) -> Self {
        Self {
            buf: &mut ep_buffer.data,
        }
    }

    pub fn alloc_endpoint(
        &mut self,
        max_packet_size: u16,
    ) -> Result<EndpointData<'d>, embassy_usb_driver::EndpointAllocError> {
        let Some((mut ep, buf)) = self
            .buf
            .split_at_mut_checked((max_packet_size as usize).next_multiple_of(4))
        else {
            return Err(EndpointAllocError);
        };

        ep = &mut ep[..max_packet_size as usize];

        //extend lifetimes to 'd
        let buf = unsafe { core::mem::transmute(buf) };
        let ep = unsafe { core::mem::transmute(ep) };
        self.buf = buf;

        Ok(EndpointData { data: ep })
    }
}

pub struct EndpointData<'d> {
    pub data: &'d mut [u8],
}

impl<'d> EndpointData<'d> {
    pub(crate) fn read_volatile(&self, buf: &mut [u8]) {
        assert!(buf.len() <= self.data.len());
        let len = buf.len();

        for i in 0..len {
            buf[i] = unsafe { core::ptr::read_volatile(&self.data[i]) };
        }
    }

    pub(crate) fn write_volatile(&mut self, buf: &[u8]) {
        assert!(buf.len() <= self.data.len());
        let len = buf.len();

        for i in 0..len {
            unsafe { core::ptr::write_volatile(&mut self.data[i], buf[i]) };
        }
    }

    pub(crate) fn addr(&self) -> usize {
        self.data.as_ptr() as usize
    }

    pub(crate) fn len(&self) -> usize {
        self.data.len()
    }
}

#[repr(C, align(4))]
pub struct EndpointBuffers<const N: usize> {
    data: [u8; N],
}

impl<const N: usize> EndpointBuffers<N> {
    pub const fn new() -> Self {
        unsafe { core::mem::zeroed() }
    }
}

impl<const N: usize> Default for EndpointBuffers<N> {
    fn default() -> Self {
        unsafe { core::mem::zeroed() }
    }
}

/// USB Direction Trait
pub trait Dir {}

/// Marker type for the "IN" direction.
pub struct In;
impl Dir for In {}

/// Marker type for the "OUT" direction.
pub struct Out;
impl Dir for Out {}

/// Marker type for the control endpoint
pub struct InOut;
impl Dir for InOut {}
