#![no_main]
#![no_std]

use core::cell::RefCell;
use core::str;

use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use arrayvec::ArrayString;
use numtoa::NumToA;

// set the panic handler
use panic_halt as _;

use stm32f4xx_hal::{
    otg_fs::{UsbBusType, USB, UsbBus}, 
    flash::{FlashExt, LockedFlash}, 
    pac::{GPIOB, Interrupt, Peripherals, interrupt, RCC, TIM2}, 
    signature::Uid, 
    gpio::{GpioExt, Output, PushPull, PC13, PC14}, 
    rcc::RccExt, 
    prelude::{_fugit_RateExtU32, _fugit_DurationExtU32},
    timer::{CounterUs, Event, TimerExt}
};

use usb_device::{
    bus::UsbBusAllocator,
    device::{UsbDevice, UsbDeviceBuilder, UsbVidPid}
};

use usbd_dfu::{DFUClass, DFUManifestationError, DFUMemError, DFUMemIO};
use usbd_webusb::{url_scheme, WebUsb};

const BOOTLOADER_SIZE_BYTES: u32 = 16 * 1024;
const KEY_STAY_IN_BOOT: u32 = 0xb0d42b89;
const FW_ADDRESS: u32 = 0x0800_4000;

// Make LED pin globally available
static LED1: Mutex<RefCell<Option<PC13<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));
static LED2: Mutex<RefCell<Option<PC14<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));

// Make timer interrupt registers globally available
static TIM: Mutex<RefCell<Option<CounterUs<TIM2>>>> = Mutex::new(RefCell::new(None));
static mut USB_DEVICE: Mutex<RefCell<Option<UsbDevice<UsbBus<USB>>>>> = Mutex::new(RefCell::new(None));
static mut USB_DFU: Mutex<RefCell<Option<DFUClass<UsbBusType, STM32Mem>>>> = Mutex::new(RefCell::new(None));
static mut USB_WUSB: Mutex<RefCell<Option<WebUsb<UsbBusType>>>> = Mutex::new(RefCell::new(None));


pub struct STM32Mem {
    flash: LockedFlash,
    buffer: [u8; 128],
}

impl<'a> STM32Mem {
    fn new(flash: LockedFlash) -> Self {
        Self {
            flash,
            buffer: [0; 128],
        }
    }
}

impl<'a> DFUMemIO for STM32Mem {
    const INITIAL_ADDRESS_POINTER: u32 = 0x0800_0000;
    const MEM_INFO_STRING: &'static str = "@Flash/0x08000000/16*1Ka,1008*1Kg";
    const HAS_DOWNLOAD: bool = true;
    const HAS_UPLOAD: bool = true;

    const MANIFESTATION_TOLERANT: bool = false;
    const PROGRAM_TIME_MS: u32 = 7;
    // time it takes to program 128 bytes
    const ERASE_TIME_MS: u32 = 50;

    const FULL_ERASE_TIME_MS: u32 = 50 * 112;

    fn store_write_buffer(&mut self, src: &[u8]) -> Result<(), ()> {
        self.buffer[..src.len()].copy_from_slice(src);
        Ok(())
    }

    fn read(&mut self, address: u32, length: usize) -> Result<&[u8], DFUMemError> {
        let flash_size_bytes: u32 = self.flash.capacity() as u32;

        let flash_top: u32 = 0x0800_0000 + flash_size_bytes as u32;

        if address < 0x0800_0000 {
            return Err(DFUMemError::Address);
        }
        if address >= flash_top {
            return Ok(&[]);
        }

        let len = length.min((flash_top - address) as usize);

        let mem = unsafe { &*core::ptr::slice_from_raw_parts(address as *const u8, len) };

        Ok(mem)
    }

    fn program(&mut self, address: u32, length: usize) -> Result<(), DFUMemError> {
        let flash_start: u32 = self.flash.address() as u32;
        let flash_size_bytes = self.flash.capacity();

        if address < flash_start {
            return Err(DFUMemError::Address);
        }

        let offset = address - flash_start;

        if offset < BOOTLOADER_SIZE_BYTES {
            return Err(DFUMemError::Address);
        }

        if offset as usize >= flash_size_bytes - length {
            return Err(DFUMemError::Address);
        }

        let mut unlocked_flash = self.flash.unlocked();
        // NorFlash::write(&mut unlocked_flash, offset, &self.buffer[..length])
        let ret = match unlocked_flash.write(offset, &self.buffer[..length]) {
            Ok(_) => Ok(()),
            Err(_) => Err(DFUMemError::Unknown),
        };
        
        drop(unlocked_flash);
        ret
    }

    fn erase(&mut self, address: u32) -> Result<(), DFUMemError> {
        let flash_start: u32 = self.flash.address() as u32;
        let flash_size_bytes: u32 = self.flash.capacity() as u32;
        
        if address < flash_start {
            return Err(DFUMemError::Address);
        }

        if address < flash_start + BOOTLOADER_SIZE_BYTES {
            return Err(DFUMemError::Address);
        }

        if address >= flash_start + flash_size_bytes as u32 {
            return Err(DFUMemError::Address);
        }

        if address & (1024 - 1) != 0 {
            return Ok(());
        }

        let mut unlocked_flash = self.flash.unlocked();
        
        let ret = match NorFlash::erase(&mut unlocked_flash, address - flash_start, address - flash_start + 1) {
            Ok(_) => Ok(()),
            Err(_) => Err(DFUMemError::Unknown),
        };
        
        drop(unlocked_flash);
        
        ret
    }

    fn erase_all(&mut self) -> Result<(), DFUMemError> {
        Err(DFUMemError::Unknown)
    }

    fn manifestation(&mut self) -> Result<(), DFUManifestationError> {
        cortex_m::interrupt::disable();

        let cortex = unsafe { cortex_m::Peripherals::steal() };

        cortex_m::asm::dsb();
        unsafe {
            // System reset request
            cortex.SCB.aircr.modify(|v| 0x05FA_0004 | (v & 0x700));
        }
        cortex_m::asm::dsb();

        loop {}
    }
}


fn dfu_init() {
    let dp = Peripherals::take().unwrap();
    
    static mut EP_MEMORY: [u32; 1024] = [0; 1024];
    static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;

    let rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_hse(25.MHz())
        .sysclk(84.MHz())
        .require_pll48clk()
        .freeze();
    let gpioa = dp.GPIOA.split();
    let gpioc = dp.GPIOC.split();

    let mut led1 = gpioc.pc13.into_push_pull_output();
    let mut led2 = gpioc.pc14.into_push_pull_output();
    let mut led3 = gpioc.pc15.into_push_pull_output();
    led1.set_high();
    led2.set_low();
    led3.set_low();

    cortex_m::interrupt::free(|cs| *LED1.borrow(cs).borrow_mut() = Some(led1));
    cortex_m::interrupt::free(|cs| *LED2.borrow(cs).borrow_mut() = Some(led2));

    let usb = USB {
        usb_global: dp.OTG_FS_GLOBAL,
        usb_device: dp.OTG_FS_DEVICE,
        usb_pwrclk: dp.OTG_FS_PWRCLK,
        pin_dm: gpioa.pa11.into_alternate(),
        pin_dp: gpioa.pa12.into_alternate(),
        hclk: clocks.hclk(),
    };
    
    

    unsafe {
        USB_BUS.replace(UsbBus::new(usb, &mut EP_MEMORY));
    }
    

    let l_flash = LockedFlash::new(dp.FLASH);
    
    let stm32mem = STM32Mem::new(l_flash);

    let usb_class = DFUClass::new(unsafe { USB_BUS.as_ref().unwrap() }, stm32mem);

    let wusb = WebUsb::new(unsafe { USB_BUS.as_ref().unwrap() }, url_scheme::HTTPS, "churrosoft.ar");

    let usb_dev = UsbDeviceBuilder::new(unsafe { USB_BUS.as_ref().unwrap() }, UsbVidPid(0xf055, 0xdf11))
        .manufacturer("Churrosoft")
        .product("Product")
        .serial_number(get_serial_str())
        .device_release(0x0200)
        .self_powered(false)
        .max_power(250)
        .max_packet_size_0(64)
        .build();

    
    unsafe {
        cortex_m::interrupt::free(|cs| {
            *USB_DFU.borrow(cs).borrow_mut() = Some(usb_class);
            *USB_WUSB.borrow(cs).borrow_mut() = Some(wusb);

            *USB_DEVICE.borrow(cs).borrow_mut() = Some(usb_dev);
        });
    }

    let mut timer = dp.TIM2.counter(&clocks);
    timer.start(1.secs()).unwrap();
    
    // Generate an interrupt when the timer expires
    timer.listen(Event::Update);
    
    // Move the timer into our global storage
    cortex_m::interrupt::free(|cs| *TIM.borrow(cs).borrow_mut() = Some(timer));
    
    unsafe {
        // enable TIM2 interrupt
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM2);
        // enable USB interrupt
        cortex_m::peripheral::NVIC::unmask(Interrupt::OTG_FS);
    }
}

#[entry]
fn main() -> ! {
    if !dfu_ram_requested() {
        minimal_init();
        if !dfu_enforced() {
            try_start_app();
        }
    }

    cortex_m::interrupt::disable();

    dfu_init();

    cortex_m::asm::dsb();
    unsafe { cortex_m::interrupt::enable() };

    loop {
        cortex_m::asm::wfi();
    }
}


/// Return true if "uninit" area of RAM has a
/// special value. Used to force DFU mode from
/// a main firmware programmatically.
fn dfu_ram_requested() -> bool {
    let stay = get_uninit_val() == KEY_STAY_IN_BOOT;
    if stay {
        clear_uninit_val();
    }
    stay
}

fn minimal_init() {
    unsafe {
        // PB2 - Input, Floating

        // Enable GPIOB clocks
        (*RCC::ptr()).ahb1enr.modify(|_, w| w.gpioben().set_bit());
        // Set to float
        (*GPIOB::ptr()).pupdr.modify(|_, w| w.pupdr2().pull_down());
        // Set to input mode
        (*GPIOB::ptr()).moder.modify(|_, w| w.moder2().input());
    }
    
    cortex_m::asm::delay(100);
}

/// Reset registers that were used for a
/// check if DFU mode must be enabled to a
/// default values before starting main firmware.
fn quick_uninit() {
    unsafe {
        // Reset every god damn register.
        (*GPIOB::ptr()).moder.reset();
        (*RCC::ptr()).ahb1enr.reset();
    }
}

/// Check if DFU force external condition.
/// Check BOOT1 jumper position.
fn dfu_enforced() -> bool {
    // check BOOT1, PB2 state
    unsafe { 
        (*GPIOB::ptr()).idr.read().idr2().bit_is_set() 
    }
}


/// Read magic value to determine if
/// device must enter DFU mode.
fn get_uninit_val() -> u32 {
    let p = 0x2000_0000 as *mut u32;
    unsafe { p.read_volatile() }
}

/// Erase magic value in RAM so that
/// DFU would be triggered only once.
fn clear_uninit_val() {
    let p = 0x2000_0000 as *mut u32;
    unsafe { p.write_volatile(0) };
}


/// Initialize stack pointer and jump to a main firmware.
#[inline(never)]
fn jump_to_app() -> ! {
    let vt = FW_ADDRESS as *const u32;
    let cortex = cortex_m::Peripherals::take().unwrap();
    
    unsafe {
        cortex.SCB.vtor.write(FW_ADDRESS & 0x3ffffe00);
        cortex_m::asm::bootload(vt);
    }
}

/// Check if FW looks OK and jump to it, or return.
fn try_start_app() {
    let sp = unsafe { (FW_ADDRESS as *const u32).read() };
    if sp & 0xfffd_0000 == 0x2000_0000 {
        quick_uninit();
        jump_to_app();
    }
}

/// Returns device serial number as hex string slice.
fn get_serial_str() -> &'static str {
    static mut SERIAL: [u8; 16] = [b' '; 16];
    let serial = unsafe { SERIAL.as_mut() };

    // let sn = read_serial();
    let uid = Uid::get();


    let mut text = ArrayString::<[_; 100]>::new();
    let mut num_buffer = [0u8; 20];

    text.push_str("C");
    text.push_str(uid.lot_num());
    text.push_str("/");
    text.push_str(uid.waf_num().numtoa_str(16, &mut num_buffer));
    text.push_str(".");
    text.push_str(uid.x().numtoa_str(16, &mut num_buffer));
    text.push_str("-");
    text.push_str(uid.y().numtoa_str(16, &mut num_buffer));

    let sn = text.as_bytes();
    
    for (i, d) in serial.iter_mut().enumerate() {
        *d = sn[i]
    }

    unsafe { str::from_utf8_unchecked(serial) }
}


fn usb_interrupt() {
    static mut L_USB_DEVICE: Option<UsbDevice<UsbBus<USB>>> = None;
    static mut L_USB_DFU: Option<DFUClass<UsbBusType, STM32Mem>> = None;
    static mut L_USB_WUSB: Option<WebUsb<UsbBusType>> = None;

    unsafe {
        let usb_dev = L_USB_DEVICE.get_or_insert_with(|| {
            cortex_m::interrupt::free(|cs| {
                // Move USB device here, leaving a None in its place
                USB_DEVICE.borrow(cs).replace(None).unwrap()
            })
        });

        let dfu = L_USB_DFU.get_or_insert_with(|| {
            cortex_m::interrupt::free(|cs| {
                // Move USB device here, leaving a None in its place
                USB_DFU.borrow(cs).replace(None).unwrap()
            })
        });

        let wusb = L_USB_WUSB.get_or_insert_with(|| {
            cortex_m::interrupt::free(|cs| {
                // Move USB device here, leaving a None in its place
                USB_WUSB.borrow(cs).replace(None).unwrap()
            })
        });
        
        usb_dev.poll(&mut [dfu, wusb]);
    }
    
}


#[interrupt]
fn OTG_FS() {
    usb_interrupt();
}

#[interrupt]
unsafe fn TIM2() {
    static mut L_TIM: Option<CounterUs<TIM2>> = None;
    static mut L_LED1: Option<PC13<Output<PushPull>>> = None;
    static mut L_LED2: Option<PC14<Output<PushPull>>> = None;
    
    let tim = L_TIM.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| TIM.borrow(cs).replace(None).unwrap() )
    });

    let led1 = L_LED1.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| LED1.borrow(cs).replace(None).unwrap())
    });

    let led2 = L_LED2.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| LED2.borrow(cs).replace(None).unwrap())
    });
    
    tim.clear_interrupt(Event::Update);
    
    led1.toggle();
    led2.toggle();
    // led.toggle().ok();
}
