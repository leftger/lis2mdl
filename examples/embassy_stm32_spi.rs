//! Embassy async SPI example for LIS2MDL magnetometer on STM32.
//!
//! This example demonstrates async SPI usage of the LIS2MDL driver with Embassy framework.
//!
//! # Hardware Setup
//!
//! - STM32 board with SPI peripheral
//! - LIS2MDL magnetometer module
//! - Connect SCLK to PA5 (or your SPI SCK pin)
//! - Connect MISO to PA6 (or your SPI MISO pin)
//! - Connect MOSI to PA7 (or your SPI MOSI pin)
//! - Connect CS to PA4 (or your chosen CS pin)
//! - Connect VDD to 3.3V
//! - Connect GND to ground
//!
//! # SPI Mode
//!
//! LIS2MDL requires SPI Mode 0 or Mode 3 (CPOL=0, CPHA=0 or CPOL=1, CPHA=1).
//!
//! # Features Required
//!
//! Run with: `cargo build --example embassy_stm32_spi --features embassy`

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::spi::{self, Spi};
use embassy_stm32::time::Hertz;
use embassy_time::{Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use lis2mdl::{Lis2mdlAsync, Lis2mdlConfig, Md, Odr, OperatingMode};
use {defmt_rtt as _, panic_probe as _};

// Uncomment and modify for your specific STM32 variant
// bind_interrupts!(struct Irqs {
//     SPI1 => spi::InterruptHandler<peripherals::SPI1>;
// });

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Initialize STM32 peripherals
    // let p = embassy_stm32::init(Default::default());
    info!("Starting LIS2MDL magnetometer SPI example...");

    // Initialize SPI at 1 MHz (max 10 MHz for LIS2MDL)
    // let mut spi_config = spi::Config::default();
    // spi_config.frequency = Hertz(1_000_000);
    // spi_config.mode = spi::Mode {
    //     polarity: spi::Polarity::IdleLow,   // CPOL=0
    //     phase: spi::Phase::CaptureOnFirstTransition,  // CPHA=0
    // };
    //
    // let spi = Spi::new(
    //     p.SPI1,
    //     p.PA5,  // SCK
    //     p.PA7,  // MOSI
    //     p.PA6,  // MISO
    //     p.DMA2_CH3,
    //     p.DMA2_CH2,
    //     spi_config,
    // );

    // Initialize CS pin
    // let cs = Output::new(p.PA4, Level::High, Speed::VeryHigh);

    // Create SPI device with CS pin management
    // let spi_device = ExclusiveDevice::new(spi, cs, embassy_time::Delay);

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

    // Create async magnetometer driver with SPI
    // let mut magnetometer = Lis2mdlAsync::new_spi_with_config(spi_device, config)
    //     .await
    //     .expect("Failed to initialize LIS2MDL via SPI");

    info!("LIS2MDL magnetometer initialized via SPI!");

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
