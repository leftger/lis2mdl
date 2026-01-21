//! Embassy async I2C example for LIS2MDL magnetometer on STM32.
//!
//! This example demonstrates async usage of the LIS2MDL driver with Embassy framework.
//!
//! # Hardware Setup
//!
//! - STM32 board with I2C peripheral
//! - LIS2MDL magnetometer module
//! - Connect SDA to PB7 (or your I2C SDA pin)
//! - Connect SCL to PB6 (or your I2C SCL pin)
//! - Connect VDD to 3.3V
//! - Connect GND to ground
//!
//! # Features Required
//!
//! Run with: `cargo build --example embassy_stm32_i2c --features embassy`

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::time::Hertz;
use embassy_time::{Duration, Timer};
use lis2mdl::{Lis2mdlAsync, Lis2mdlConfig, Md, Odr, OperatingMode};
use {defmt_rtt as _, panic_probe as _};

// Uncomment and modify for your specific STM32 variant
// bind_interrupts!(struct Irqs {
//     I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
//     I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
// });

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Initialize STM32 peripherals
    // let p = embassy_stm32::init(Default::default());
    info!("Starting LIS2MDL magnetometer example...");

    // Initialize I2C at 400 kHz
    // let i2c = I2c::new(
    //     p.I2C1,
    //     p.PB6,  // SCL
    //     p.PB7,  // SDA
    //     Irqs,
    //     p.DMA1_CH6,
    //     p.DMA1_CH7,
    //     Hertz(400_000),
    //     Default::default(),
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

    // Create async magnetometer driver
    // let mut magnetometer = Lis2mdlAsync::new_i2c_with_config(i2c, config)
    //     .await
    //     .expect("Failed to initialize LIS2MDL");

    info!("LIS2MDL magnetometer initialized!");

    loop {
        // Read magnetic field in gauss
        // match magnetometer.read_gauss().await {
        //     Ok(field) => {
        //         info!(
        //             "Magnetic field: X={:.3} Y={:.3} Z={:.3} gauss",
        //             field.x, field.y, field.z
        //         );
        //
        //         // Calculate magnitude
        //         let magnitude = (field.x * field.x + field.y * field.y + field.z * field.z).sqrt();
        //         info!("Magnitude: {:.3} gauss", magnitude);
        //     }
        //     Err(_) => error!("Error reading magnetometer"),
        // }

        // Read temperature
        // match magnetometer.read_temperature().await {
        //     Ok(temp) => info!("Temperature: {:.1}°C", temp.to_celsius()),
        //     Err(_) => error!("Error reading temperature"),
        // }

        // Wait before next reading
        Timer::after(Duration::from_millis(100)).await;
    }
}
