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
    let clocks = rcc.cfgr.sysclk(8.MHz()).freeze(&mut flash.acr); // Configure clocks and freeze them.  Need 8 MHz for ITM?.


    // Create an LED output pin on PE13.
    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);
    let mut led = gpioe.pe13.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    led.set_low().unwrap();

    // Configure pins on port B, where the I2C peripheral is.
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
    let i2c_pins = (
        gpiob.pb6.into_af4_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl), // SCL
        gpiob.pb7.into_af4_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl), // SDA
    );
    let mut my_i2c = hal::i2c::I2c::new(dp.I2C1, i2c_pins, 100_000.Hz(), clocks, &mut rcc.apb1);

    let mut mysensor = sts3x::Sts3x::new(my_i2c, sts3x::PeripheralAddr::PinLow);
    

//    let mut i = 0;
    loop {
        // Read temperature data from STS31
        // my_i2c.write(STS31_ADDR, &[0x24, 0x00]).unwrap(); // no clock stretching, high repeatability
        // let mut meas = [0u8; 3];
        // let mut read_res = my_i2c.read(STS31_ADDR, &mut meas);
        // loop {
        //     match read_res {
        //         Ok(y) => {
        //             let foo: f64 = -45.0 + 175.0 * (meas[0] as f64 * 256.0 + meas[1] as f64) / 65535.0;
        //             iprintln!(stim, "OK val {:?} and temp {}", y, foo);
        //             break;
        //         },
        //         Err(hal::i2c::Error::Nack) => { // this is stm32f3xx_hal::i2c::Error::Nack
        //             // Going the path through stm32 into its reexported embedded_hal: hal::hal::blocking::i2c::Read::Error, with no Nack.
        //             iprintln!(stim, "err is Nack");
        //             asm::delay(8_000); // 1 msec
        //             read_res = my_i2c.read(STS31_ADDR, &mut meas);
        //         },
        //         Err(ref e) => {
        //             iprintln!(stim, "err {:?}", e);
        //             asm::delay(8_000); // 1 msec
        //             read_res = my_i2c.read(STS31_ADDR, &mut meas);
        //         }
        //     }
        // }
        mysensor.trigger_temp_meas().unwrap();
        asm::delay(8_000); // 1 msec
        loop {
            let temp = mysensor.read_temperature();
            match temp {
                Ok(x) => {
                    iprintln!(stim, "temp: {}", x);
                    break;
                },
                Err(ref e) => {
                    iprintln!(stim, "err {:?}", e);
                }
            }
            asm::delay(8_000); // 1 msec
        }
 //        let foo: f64 = -45.0 + 175.0 * (meas[0] as f64 * 256.0 + meas[1] as f64) / 65535.0;
//        iprintln!(stim, "{:?} {:?} {:?} {} C", meas, crc::crc8(&[meas[0], meas[1]]), crc::is_crc8_valid(&meas), foo);
//        let temp = mysensor.read_temperature().unwrap();
//        iprintln!(stim, "temp: 0x{:x}", temp);

        asm::delay(8_000_000);
        led.toggle().unwrap();
    }
}
