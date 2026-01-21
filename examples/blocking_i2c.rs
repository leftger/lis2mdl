//! Blocking I2C example for LIS2MDL magnetometer.
//!
//! This example demonstrates basic usage of the LIS2MDL driver with a blocking I2C interface.
//!
//! # Hardware Setup
//!
//! - Connect SDA to your MCU's I2C SDA pin
//! - Connect SCL to your MCU's I2C SCL pin
//! - Connect VDD to 3.3V or 2.5V
//! - Connect GND to ground
//!
//! # Note
//!
//! This is a template example. You'll need to adapt it for your specific hardware platform.

#![no_std]
#![no_main]

use core::fmt::Write;
use lis2mdl::{Lis2mdl, Lis2mdlConfig, Md, Odr, OperatingMode};
use panic_halt as _;

// Replace these with your platform-specific imports
// use your_hal as hal;
// use hal::i2c::I2c;
// use hal::prelude::*;

#[entry]
fn main() -> ! {
    // Initialize your hardware (example for a generic HAL)
    // let peripherals = hal::Peripherals::take().unwrap();
    // let mut delay = hal::Delay::new();

    // Initialize I2C
    // let i2c = hal::I2c::new(
    //     peripherals.I2C1,
    //     peripherals.GPIO_SDA,
    //     peripherals.GPIO_SCL,
    //     400.kHz(),
    // );

    // Configure the magnetometer
    let config = Lis2mdlConfig {
        odr: Odr::Hz100,                     // 100 Hz output data rate
        mode: OperatingMode::HighResolution, // High resolution mode
        measurement_mode: Md::Continuous,    // Continuous measurement
        temperature_compensation: true,      // Enable temp compensation
        block_data_update: true,             // Enable BDU
        low_pass_filter: false,              // Disable low-pass filter
        ..Default::default()
    };

    // Create magnetometer driver
    // let mut magnetometer = Lis2mdl::new_i2c_with_config(i2c, config)
    //     .expect("Failed to initialize LIS2MDL");

    // println!("LIS2MDL magnetometer initialized!");

    loop {
        // Wait for new data
        // if magnetometer.data_ready().unwrap_or(false) {
        //     // Read magnetic field in gauss
        //     match magnetometer.read_gauss() {
        //         Ok(field) => {
        //             println!(
        //                 "Magnetic field: X={:.3} Y={:.3} Z={:.3} gauss",
        //                 field.x, field.y, field.z
        //             );
        //
        //             // Calculate magnitude
        //             let magnitude = (field.x * field.x + field.y * field.y + field.z * field.z).sqrt();
        //             println!("Magnitude: {:.3} gauss", magnitude);
        //         }
        //         Err(e) => println!("Error reading magnetometer: {:?}", e),
        //     }
        //
        //     // Read temperature
        //     match magnetometer.read_temperature() {
        //         Ok(temp) => println!("Temperature: {:.1}°C", temp.to_celsius()),
        //         Err(e) => println!("Error reading temperature: {:?}", e),
        //     }
        //
        //     println!("---");
        // }

        // delay.delay_ms(10);
    }
}
