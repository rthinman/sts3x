#![no_std]
#![no_main]
#![deny(unsafe_code)]

// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// use panic_abort as _; // requires nightly
// use panic_itm as _; // logs messages over ITM; requires ITM support
// use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use cortex_m::asm;
use cortex_m_rt::entry;
use cortex_m::{iprintln, Peripherals};

use stm32f3xx_hal as hal;

use hal::pac;
use hal::prelude::*;

use sts3x;

const STS31_ADDR: u8 = 0x4A;

#[entry]
fn main() -> ! {
    let mut p = Peripherals::take().unwrap();  // Cortex core peripherals
    let stim = &mut p.ITM.stim[0];
    iprintln!(stim, "Hello  world!");

    // Get device peripherals and set clocks.
    let dp = pac::Peripherals::take().unwrap(); // Device peripherals
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(8.mhz()).freeze(&mut flash.acr); // Configure clocks and freeze them.  Need 8 MHz for ITM?.


    // Create an LED output pin on PE13.
    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);
    let mut led = gpioe.pe13.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    led.set_low().unwrap();

    // Configure pins on port B, where the I2C peripheral is.
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
    let i2c_pins = (
        gpiob.pb6.into_af4(&mut gpiob.moder, &mut gpiob.afrl), // SCL
        gpiob.pb7.into_af4(&mut gpiob.moder, &mut gpiob.afrl), // SDA
    );
    let mut my_i2c = hal::i2c::I2c::new(dp.I2C1, i2c_pins, 100.khz(), clocks, &mut rcc.apb1);
    

    let mut i = 0;
    loop {
        // Read temperature data from STS31
        my_i2c.write(STS31_ADDR, &[0x24, 0x00]).unwrap(); // no clock stretching, high repeatability
        asm::delay(16 * 8_000); // 16 msec
        let mut meas = [0u8; 3];
        my_i2c.read(STS31_ADDR, &mut meas).unwrap();
        let foo: f64 = -45.0 + 175.0 * (meas[0] as f64 * 256.0 + meas[1] as f64) / 65535.0;
        iprintln!(stim, "{:?} {:?} {:?} {} C", meas, crc::crc8(&[meas[0], meas[1]]), crc::is_crc8_valid(&meas), foo);

        asm::delay(8_000_000);
        i = sts3x::add_one(&i);
        led.toggle().unwrap();
    }
}

mod crc {
/// Functions to calculate the CRC8 checksum and validate it
///
/// Implementation based on the shtcx crate, which was based on the reference by Sensirion.
//pub(crate) fn crc8(data: &[u8]) -> u8 {
    pub fn crc8(data: &[u8]) -> u8 {
        const CRC8_POLYNOMIAL: u8 = 0x31;
        let mut crc: u8 = 0xFF;
        for byte in data {
            crc ^= byte;
            for _ in 0..8 {
                if (crc & 0x80) > 0 {
                    crc = (crc << 1) ^ CRC8_POLYNOMIAL;
                } else {
                    crc <<= 1;
                }
            }
        }
        crc
    }

/// Validate the checksum.  data should contain the data + checksum as the last byte.
//pub(crate) fn is_crc8_valid(data: &[u8]) -> bool {
    pub fn is_crc8_valid(data: &[u8]) -> bool {
        crc8(data) == 0
    }
}
