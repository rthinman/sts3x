//! This is a platform-agnostic Rust driver for the Sensirion STS30, STS31, and STS35
//! high-accuracy, low-power, I2C digital temperature sensors, based on the 
//! [`embedded-hal`] traits.
//! 
//! [`embedded-hal`]: https://github.com/rust-embedded/embedded-hal
//! 
//! TODO: More information here.
//! 
//! The driver borrows liberally from: 
//! - eldruin's tmp1x2-rs driver for Texas Instruments TMP102 and TMP112, https://github.com/eldruin/tmp1x2-rs, and
//! - dbrgn's shtcx-rs driver for Sensirion SHTCx temperature/humidity sensors, https://github.com/dbrgn/shtcx-rs.

#![deny(unsafe_code)]
#![no_std]

// TODO: add deny missing docs, and doc root url

mod crc;

use core::marker::PhantomData;
use embedded_hal::blocking::i2c;  // TODO: move to using nb if the crate adds a nonblocking I2C.
pub use nb;

/// Possible errors in this crate
#[derive(Debug)]
pub enum Error<E> {
    /// I²C bus error
    I2C(E),
    /// CRC checksum validation failed
    Crc,
}

/// Error type for mode changes.
/// This allows us to retrieve the unchanged device in case of an error.
#[derive(Debug)]
pub enum ModeChangeError<E, DEV> {
    /// I²C bus error while changing modes
    /// 
    /// `E` is the error that happened.
    /// `DEV` is the device with the mode unchanged.
    I2C(E, DEV),
}

/// Conversion rate for continuous conversion mode.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ConversionRate {
    /// 0.5 Hz
    _0_5Hz,
    /// 1 Hz
    _1Hz,
    /// 2 Hz
    _2Hz,
    /// 4 Hz
    _4Hz,
    /// 10 Hz
    _10Hz,
}

/// Repeatability condition for both one-shot and continuous modes.
/// From the datasheet: the value is 3 * standard deviation of measurements at constant ambient.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Repeatability {
    /// High repeatability 0.04°C
    High,
    /// Medium repeatability 0.08°C
    Medium,
    /// Low repeatability 0.15°C
    Low,
}

impl Default for Repeatability {
    fn default() -> Self {
        Repeatability::Low
    }
}

/// Possible peripheral addresses
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PeripheralAddr {
    /// Default address, with address pin held low
    PinLow,
    /// Address with the pin held high
    PinHigh,
}

impl Default for PeripheralAddr {
    fn default() -> Self {
        PeripheralAddr::PinLow
    }
}

impl PeripheralAddr {
    /// Return the 7-bit I2C address corresponding to the enum.
    fn as_byte(self) -> u8 {
        match self {
            PeripheralAddr::PinLow => 0x4A,
            PeripheralAddr::PinHigh => 0x4B,
        }
    }
}

/// I²C commands sent to the sensor
#[derive(Debug)]
enum Command {
    /// Initiate a single-shot conversion.
    StartSingleShot {repeatability: Repeatability},
    /// Change to periodic mode with the given repeatability and conversion rates.
    StartPeriodic {
        repeatability: Repeatability,
        conversion_rate: ConversionRate,
    },
    /// Fetch data from the sensor when it is in continuous mode.
    FetchData,
    /// Break out of continuous mode and return to one-shot mdoe.
    Break,
    /// Issue a software reset.
    SoftReset,
    /// Turn the heater on for plausibility checking.
    HeaterOn,
    /// Turn the heater off.
    HeaterOff,
    /// Read the status register.
    ReadStatus,
    /// Clear the status register.
    ClearStatus,
}

impl Command {
    /// Return a slice of two bytes corresponding to the command. 
    /// These are the bytes the sensor expects.
    fn as_bytes(self) -> [u8; 2] {
        match self {
            // For single shot, listing each variant directly. 
            Command::StartSingleShot{repeatability: Repeatability::High} => [0x24, 0x00],
            Command::StartSingleShot{repeatability: Repeatability::Medium} => [0x24, 0x0B],
            Command::StartSingleShot{repeatability: Repeatability::Low} => [0x24, 0x16],
            // For periodic, using nested matches, more lines of code, but hopefully more readable. 
            Command::StartPeriodic{repeatability: r, conversion_rate: c} => {
                match c {
                    ConversionRate::_0_5Hz => {
                        match r {
                            Repeatability::High => [0x20, 0x32],
                            Repeatability::Medium => [0x20, 0x24],
                            Repeatability::Low => [0x20, 0x2F],
                        }
                    },
                    ConversionRate::_1Hz => {
                        match r {
                            Repeatability::High => [0x21, 0x30],
                            Repeatability::Medium => [0x21, 0x26],
                            Repeatability::Low => [0x21, 0x2D],
                        }
                    },
                    ConversionRate::_2Hz => {
                        match r {
                            Repeatability::High => [0x22, 0x36],
                            Repeatability::Medium => [0x22, 0x20],
                            Repeatability::Low => [0x22, 0x2B],
                        }
                    },
                    ConversionRate::_4Hz => {
                        match r {
                            Repeatability::High => [0x23, 0x34],
                            Repeatability::Medium => [0x23, 0x22],
                            Repeatability::Low => [0x23, 0x29],
                        }
                    },
                    ConversionRate::_10Hz => {
                        match r {
                            Repeatability::High => [0x27, 0x37],
                            Repeatability::Medium => [0x27, 0x21],
                            Repeatability::Low => [0x27, 0x2A],
                        }
                    },
                }
            },
            Command::FetchData => [0xE0, 0x00],
            Command::Break => [0x30, 0x93],
            Command::SoftReset => [0x30, 0xA2],
            Command::HeaterOn => [0x30, 0x6D],
            Command::HeaterOff => [0x30, 0x66],
            Command::ReadStatus => [0xF3, 0x2D],
            Command::ClearStatus => [0x30, 0x41],
        }
    }
}

// TODO: this is from shtcx, and I need to figure out how/whether to implement
pub trait MeasurementDuration {
    /// Return the maximum measurement duration according to repeatability
    /// in microseconds
    fn measurement_duration_us(repeat: Repeatability) -> u16;
}

#[doc(hidden)]
// Type states for one-shot and continuous modes.
pub mod marker {
    pub mod mode {
        #[derive(Debug)]
        pub struct OneShot(());
        #[derive(Debug)]
        pub struct Continuous(());
    }
}

/// Device Driver
#[derive(Debug, Default)]
pub struct Sts3x<I2C, MODE> {
    /// The I2C device implementation
    i2c: I2C,
    /// The 7-bit I2C device address
    address: u8,
    /// The present repeatabiliy setting
    repeatability: Repeatability,
    /// A temperature measurement was started.
    temp_measurement_started: bool,
    _mode: PhantomData<MODE>,
}

// Implement struct creation for OneShot only so that it is only possible to create a one-shot version.
impl<I2C, E> Sts3x<I2C, marker::mode::OneShot>
where
    I2C: i2c::Write<Error = E>,
{
    /// Create new instance of the Sts3x device.
    /// Defaults to Low repeatability for power savings.
    /// Change repeatability with set_repeatability().
    /// 
    /// By default, the device starts in one-shot mode.
    pub fn new(i2c: I2C, address: PeripheralAddr) -> Self {
        Sts3x {
            i2c,
            address: address.as_byte(),
            repeatability: Repeatability::default(),
            temp_measurement_started: false,
            _mode: PhantomData,
        }
    }

    /// Create new instance of the Sts3x device, choosing a Repeatability.
    /// 
    /// By default, the device starts in one-shot mode.
    pub fn new_with_repeatability(i2c: I2C, address: PeripheralAddr, repeatability: Repeatability) -> Self {
        Sts3x {
            i2c,
            address: address.as_byte(),
            repeatability,
            temp_measurement_started: false,
            _mode: PhantomData,
        }
    }
}

// Methods shared by both single-shot and continuous modes
impl<I2C, MODE, E> Sts3x<I2C, MODE> 
where
    I2C: i2c::Read<Error = E> + i2c::Write<Error = E>,
{
    /// Destroy driver instance and return the I2C bus instance.
    pub fn destroy(self) -> I2C {
        self.i2c
    }

    // write and read private methods

    /// Write an I2C command to the sensor
    fn send_command(&mut self, command: Command) -> Result<(), Error<E>> {
        self.i2c
            .write(self.address, &command.as_bytes())
            .map_err(Error::I2C)
    }

    /// Read and check the CRC.
    /// Returns a Result with u16 corresponding to the MSB,LSB of the first
    /// two bytes of the buffer. 
    fn read_with_crc(&mut self) -> Result<u16, Error<E>> {
        let mut buf = [0u8; 3];
        self.i2c.read(self.address, &mut buf).map_err(Error::I2C)?;
        if crc::is_crc8_valid(&buf) {
            let x: u16 = (buf[0] as u16) << 8 | (buf[1] as u16);
            Ok(x)
        } else {
            Err(Error::Crc)
        }
    }

    fn convert_temp_to_float(temp: u16) -> f32 {
        -45.0 + 175.0 * (temp as f32) / 65535.0
    }

    // method about measurement duration?
}

// Methods for one-shot mode only
// TODO: these are nonblocking, but don't utilize the nb concepts.  Also make blocking types.
impl<I2C, E> Sts3x<I2C, marker::mode::OneShot>
where
    I2C: i2c::Read<Error = E> + i2c::Write<Error = E>,
{
    /// Start a one-shot temperature measurement using the repeatability
    /// that has already been set.   
    pub fn trigger_temp_meas(&mut self) -> Result<(), Error<E>> {
        self.send_command(Command::StartSingleShot{repeatability: self.repeatability})
    }

    /// Perform a one-shot temperature measurement.
    ///
    /// This allows triggering a single temperature measurement when in
    /// one-shot mode. The device returns to the low-power state at the
    /// completion of the temperature conversion, reducing power
    /// consumption when continuous temperature monitoring is not required.
    ///
    /// If no temperature conversion was started yet, calling this method
    /// will start one and return `nb::Error::WouldBlock`. Subsequent calls
    /// will continue to return `nb::Error::WouldBlock` until the
    /// temperature measurement is finished. Then it will return the
    /// measured temperature in °C.
    pub fn read_temperature(&mut self) -> nb::Result<f32, Error<E>> {
        if !self.temp_measurement_started {
            self.trigger_temp_meas()
                .map_err(nb::Error::Other)?;
            self.temp_measurement_started = true;
            return Err(nb::Error::WouldBlock);
        }
        let mut buf = [0u8; 3];
        let completion = self.i2c.read(self.address, &mut buf);

        // What I want to do:
        // match completion {
        //     Ok(val) => {
        //         // Conversion complete.
        //         let x: u16 = (buf[0] as u16) << 8 | (buf[1] as u16);
        //         self.temp_measurement_started = false;
        //         Ok(Self::convert_temp_to_float(x))
        //     },
        //     Err(stm32f3xx_hal::i2c::Error::Nack) => { // I want to replace with a generic path in embedded_hal
        //                                               // namespace because we shouldn't depend on a specific device HAL.  
        //         Err(nb::Error::WouldBlock)
        //     },
        //     Err(e) => {
        //         Err(nb::Error::Other(Error::I2C(e))) // Not sure this is correct, but compiler doesn't complain.
        //     }
        // }

        // What I have to do with embedded_hal 0.2.4/0.2.5:
        match completion {
            Ok(_) => {
                // Conversion complete.
                self.temp_measurement_started = false;
                if crc::is_crc8_valid(&buf) {
                    let x: u16 = (buf[0] as u16) << 8 | (buf[1] as u16);
                    Ok(Self::convert_temp_to_float(x))
                } else {
                    Err(nb::Error::Other(Error::Crc))
                }
            },
            _ => {
                Err(nb::Error::WouldBlock)
            }
        }
    }

    pub fn set_repeatability(&mut self, r: Repeatability) -> Repeatability{
        self.repeatability = r;
        r
    }

    // pub fn into_continuous(self, rate: ConversionRate) -> Result<Sts3x<I2C, marker::mode::Continuous>, ModeChangeError<E, Self>> 

    // /// Reset the state of the driver, to be used if there was a "general call" on the I2C bus.
    // pub fn reset_state(&mut self)
    // pub fn soft_reset(&mut self)
    // pub fn get_status(&self)
    // pub fn clear_status(&mut self)
    // pub fn heater_on(&mut self)
    // pub fn heater_off(&mut self)

}


// Methods for continuous mode only
impl<I2C, E> Sts3x<I2C, marker::mode::Continuous>
where
    I2C: i2c::Write<Error = E>,
{
    /// Get latest temperature reading.
    /// TODO: fill out
    pub fn read_temperature(&self) -> u16 {
        25
    }

    // /// Convert to one-shot mode.
    // /// TODO: add the command to change and a failure error.
    // pub fn into_one_shot(self) -> Result<Sts3x<I2C, marker::mode::OneShot>, ModeChangeError<E, Self>> {
    //     Result(Sts3x {
    //         i2c: self.i2c,
    //         address: self.address,
    //         repeatability: self.repeatability,
    //         temp_measurement_started: false,
    //         _mode: PhantomData,
    //     })
    // }

    // /// Reset the state of the driver, to be used if there was a "general call" on the I2C bus.
    // /// This will convert into a one-shot mode device.
    // pub fn reset_state(mut self)

}



// impl MeasurementDuration for Sts3x<I2C, MODE> {
//     // TODO: fill out fn measurement_duration
//     fn measurement_duration_us(repeat: Repeatability) -> u16 {
//         20
//     }
// }

