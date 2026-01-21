//! Hard-iron calibration example for LIS2MDL magnetometer.
//!
//! This example demonstrates how to perform hard-iron calibration on the LIS2MDL.
//!
//! Hard-iron calibration compensates for constant magnetic field offsets caused by
//! ferromagnetic materials near the sensor (e.g., speakers, motors, metal chassis).
//!
//! # Calibration Procedure
//!
//! 1. Place the sensor in a location away from strong magnetic fields
//! 2. Slowly rotate the sensor through all possible orientations
//! 3. The code will collect min/max values for each axis
//! 4. Calculate offset as the midpoint between min and max
//! 5. Apply the offset to future readings
//!
//! # Hardware Setup
//!
//! - Connect SDA to your MCU's I2C SDA pin
//! - Connect SCL to your MCU's I2C SCL pin
//! - Connect VDD to 3.3V or 2.5V
//! - Connect GND to ground

#![no_std]
#![no_main]

use core::fmt::Write;
use lis2mdl::{Lis2mdl, Lis2mdlConfig, Md, Odr, OperatingMode};
use panic_halt as _;

// Replace these with your platform-specific imports
// use your_hal as hal;
// use hal::i2c::I2c;
// use hal::prelude::*;

const CALIBRATION_SAMPLES: usize = 200;

#[entry]
fn main() -> ! {
    // Initialize your hardware
    // let peripherals = hal::Peripherals::take().unwrap();
    // let mut delay = hal::Delay::new();

    // Initialize I2C
    // let i2c = hal::I2c::new(
    //     peripherals.I2C1,
    //     peripherals.GPIO_SDA,
    //     peripherals.GPIO_SCL,
    //     400.kHz(),
    // );

    // Configure the magnetometer for calibration
    let config = Lis2mdlConfig {
        odr: Odr::Hz100,                     // 100 Hz for faster sampling
        mode: OperatingMode::HighResolution, // High resolution for accuracy
        measurement_mode: Md::Continuous,    // Continuous measurement
        temperature_compensation: true,      // Enable temp compensation
        block_data_update: true,             // Enable BDU
        ..Default::default()
    };

    // Create magnetometer driver without calibration
    // let mut magnetometer = Lis2mdl::new_i2c_with_config(i2c, config)
    //     .expect("Failed to initialize LIS2MDL");

    // println!("=== LIS2MDL Hard-Iron Calibration ===");
    // println!("Rotate the sensor slowly through all orientations...");
    // println!("Collecting {} samples...", CALIBRATION_SAMPLES);

    // Initialize min/max tracking
    let mut min = [i16::MAX; 3];
    let mut max = [i16::MIN; 3];

    // Collect calibration samples
    // for i in 0..CALIBRATION_SAMPLES {
    //     // Wait for new data
    //     while !magnetometer.data_ready().unwrap_or(false) {
    //         delay.delay_ms(1);
    //     }
    //
    //     // Read raw magnetic field data
    //     match magnetometer.read_raw() {
    //         Ok(field) => {
    //             // Update min/max for each axis
    //             min[0] = min[0].min(field.x);
    //             min[1] = min[1].min(field.y);
    //             min[2] = min[2].min(field.z);
    //
    //             max[0] = max[0].max(field.x);
    //             max[1] = max[1].max(field.y);
    //             max[2] = max[2].max(field.z);
    //
    //             // Print progress every 20 samples
    //             if (i + 1) % 20 == 0 {
    //                 println!("Progress: {}/{}", i + 1, CALIBRATION_SAMPLES);
    //             }
    //         }
    //         Err(e) => println!("Error reading magnetometer: {:?}", e),
    //     }
    // }

    // Calculate hard-iron offset (midpoint between min and max)
    // let offset = [
    //     (min[0] + max[0]) / 2,
    //     (min[1] + max[1]) / 2,
    //     (min[2] + max[2]) / 2,
    // ];

    // println!("\n=== Calibration Results ===");
    // println!("Min values: [{}, {}, {}]", min[0], min[1], min[2]);
    // println!("Max values: [{}, {}, {}]", max[0], max[1], max[2]);
    // println!("Calculated offset: [{}, {}, {}]", offset[0], offset[1], offset[2]);

    // Apply the calibration offset
    // magnetometer.set_offset(offset).expect("Failed to set offset");
    // println!("Offset applied to sensor!");

    // Verify calibration by reading offset back
    // match magnetometer.get_offset() {
    //     Ok(read_offset) => {
    //         println!("Verified offset: [{}, {}, {}]",
    //             read_offset[0], read_offset[1], read_offset[2]);
    //     }
    //     Err(e) => println!("Error reading offset: {:?}", e),
    // }

    // println!("\n=== Normal Operation (with calibration) ===");
    // println!("Reading calibrated magnetic field...\n");

    // Normal operation with calibrated readings
    loop {
        // Wait for new data
        // if magnetometer.data_ready().unwrap_or(false) {
        //     // Read calibrated magnetic field in gauss
        //     match magnetometer.read_gauss() {
        //         Ok(field) => {
        //             // Calculate magnitude
        //             let magnitude = (field.x * field.x + field.y * field.y + field.z * field.z).sqrt();
        //
        //             println!(
        //                 "Field: X={:.3} Y={:.3} Z={:.3} | Mag={:.3} gauss",
        //                 field.x, field.y, field.z, magnitude
        //             );
        //         }
        //         Err(e) => println!("Error: {:?}", e),
        //     }
        // }

        // delay.delay_ms(100);
    }
}
