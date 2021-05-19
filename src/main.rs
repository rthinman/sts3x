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

#[entry]
fn main() -> ! {
    let mut p = Peripherals::take().unwrap();  // Cortex core peripherals
    let stim = &mut p.ITM.stim[0];
    iprintln!(stim, "Hello  world!");

    // Create an LED output pin on PE13.
    let dp = pac::Peripherals::take().unwrap(); // Device peripherals
    let mut rcc = dp.RCC.constrain();
    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);
    let mut led = gpioe.pe13.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    led.set_low().unwrap();

    let mut i = 0;
    loop {
        iprintln!(stim, "test {}", i);
        asm::delay(8_000_000);
        i += 1;
        led.toggle().unwrap();
    }
}
