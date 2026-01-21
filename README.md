# LIS2MDL Driver

Platform-agnostic Rust driver for the [LIS2MDL](https://www.st.com/en/mems-and-sensors/lis2mdl.html) 3-axis digital magnetometer from STMicroelectronics.

[![Crates.io](https://img.shields.io/crates/v/lis2mdl.svg)](https://crates.io/crates/lis2mdl)
[![Documentation](https://docs.rs/lis2mdl/badge.svg)](https://docs.rs/lis2mdl)

## Features

- ✅ Blocking and async API support
- ✅ I2C and SPI interfaces
- ✅ Hard-iron offset calibration
- ✅ Temperature compensation
- ✅ Low-pass filtering
- ✅ Self-test capability
- ✅ Configurable output data rate (10-100 Hz)
- ✅ High-resolution and low-power modes
- ✅ `no_std` compatible
- ✅ Optimized for Embassy async framework

## Device Overview

The LIS2MDL is an ultra-low-power, high-performance 3-axis digital magnetic sensor with:

- ±50 gauss magnetic field full scale
- 16-bit data output
- Continuous and single-shot measurement modes
- Integrated temperature sensor
- I2C (400 kHz) and SPI (up to 10 MHz) interfaces
- 2.5V to 3.6V supply voltage
- Ultra-low power consumption: 50 μA (low-power), 200 μA (high-resolution)

## Usage

Add this to your `Cargo.toml`:

```toml
[dependencies]
lis2mdl = "0.1"
```

For async support:

```toml
[dependencies]
lis2mdl = { version = "0.1", features = ["async"] }
```

For Embassy framework:

```toml
[dependencies]
lis2mdl = { version = "0.1", features = ["embassy"] }
```

## Examples

### Blocking I2C

```rust
use lis2mdl::{Lis2mdl, Lis2mdlConfig, Md, Odr, OperatingMode};

// Initialize I2C
let i2c = /* your I2C peripheral */;

// Configure magnetometer
let config = Lis2mdlConfig {
    odr: Odr::Hz100,
    mode: OperatingMode::HighResolution,
    measurement_mode: Md::Continuous,
    temperature_compensation: true,
    ..Default::default()
};

// Create driver
let mut magnetometer = Lis2mdl::new_i2c_with_config(i2c, config)?;

// Read magnetic field
let field = magnetometer.read_gauss()?;
println!("Magnetic field: x={:.3}, y={:.3}, z={:.3} gauss",
    field.x, field.y, field.z);

// Read temperature
let temp = magnetometer.read_temperature()?;
println!("Temperature: {:.1}°C", temp.to_celsius());
```

### Async I2C with Embassy

```rust
use lis2mdl::{Lis2mdlAsync, Lis2mdlConfig};

// Initialize async I2C
let i2c = /* your async I2C peripheral */;

// Create async driver
let mut magnetometer = Lis2mdlAsync::new_i2c(i2c).await?;

// Read magnetic field asynchronously
let field = magnetometer.read_gauss().await?;
```

### Hard-Iron Calibration

```rust
// Collect samples while rotating sensor through all orientations
let mut min = [i16::MAX; 3];
let mut max = [i16::MIN; 3];

for _ in 0..200 {
    let field = magnetometer.read_raw()?;
    min[0] = min[0].min(field.x);
    min[1] = min[1].min(field.y);
    min[2] = min[2].min(field.z);
    max[0] = max[0].max(field.x);
    max[1] = max[1].max(field.y);
    max[2] = max[2].max(field.z);
}

// Calculate and apply offset
let offset = [
    (min[0] + max[0]) / 2,
    (min[1] + max[1]) / 2,
    (min[2] + max[2]) / 2,
];
magnetometer.set_offset(offset)?;
```

### SPI Interface

```rust
use lis2mdl::Lis2mdl;

// Initialize SPI
let spi = /* your SPI device with CS management */;

// Create driver with SPI
let mut magnetometer = Lis2mdl::new_spi(spi)?;

// Use the same API as I2C
let field = magnetometer.read_gauss()?;
```

## Configuration Options

### Operating Modes

- **High Resolution**: 200 μA, maximum accuracy
- **Low Power**: 50 μA, reduced power consumption

### Output Data Rates

- 10 Hz
- 20 Hz
- 50 Hz
- 100 Hz

### Measurement Modes

- **Continuous**: Automatic periodic measurements
- **Single**: One-shot measurement on demand
- **Idle**: Power-down mode

### Additional Features

- **Temperature Compensation**: Corrects magnetic readings for temperature drift
- **Low-Pass Filter**: Reduces noise by filtering at ODR/4 instead of ODR/2
- **Offset Cancellation**: Automatic or manual hard-iron offset correction
- **Block Data Update**: Ensures MSB and LSB are from the same sample
- **Self-Test**: Built-in hardware self-test for validation

## API Overview

### Blocking API

```rust
impl<IFACE> Lis2mdl<IFACE> {
    // Constructors
    fn new_i2c(i2c: I2C) -> Result<Self, Error>;
    fn new_spi(spi: SPI) -> Result<Self, Error>;
    fn new_i2c_with_config(i2c: I2C, config: Lis2mdlConfig) -> Result<Self, Error>;
    fn new_spi_with_config(spi: SPI, config: Lis2mdlConfig) -> Result<Self, Error>;

    // Data reading
    fn read_raw(&mut self) -> Result<RawMagneticField, Error>;
    fn read_gauss(&mut self) -> Result<GaussMagneticField, Error>;
    fn read_temperature(&mut self) -> Result<Temperature, Error>;

    // Status
    fn status(&mut self) -> Result<Status, Error>;
    fn data_ready(&mut self) -> Result<bool, Error>;

    // Calibration
    fn set_offset(&mut self, offset: [i16; 3]) -> Result<(), Error>;
    fn get_offset(&mut self) -> Result<[i16; 3], Error>;

    // Configuration
    fn set_config(&mut self, config: Lis2mdlConfig) -> Result<(), Error>;
    fn config(&self) -> &Lis2mdlConfig;

    // Device control
    fn enable_self_test(&mut self, enable: bool) -> Result<(), Error>;
    fn soft_reset(&mut self) -> Result<(), Error>;
    fn reboot(&mut self) -> Result<(), Error>;
}
```

### Async API

All methods from the blocking API are available as async versions:

```rust
impl<IFACE> Lis2mdlAsync<IFACE> {
    async fn new_i2c(i2c: I2C) -> Result<Self, Error>;
    async fn read_gauss(&mut self) -> Result<GaussMagneticField, Error>;
    // ... etc
}
```

## Hardware Connections

### I2C Interface

| LIS2MDL Pin | MCU Pin | Description |
|-------------|---------|-------------|
| VDD         | 3.3V    | Power supply (2.5V - 3.6V) |
| GND         | GND     | Ground |
| SDA         | SDA     | I2C data line |
| SCL         | SCL     | I2C clock line |
| CS          | VDD     | Tie high for I2C mode |

**I2C Address**: 0x1E (fixed, not configurable)

### SPI Interface

| LIS2MDL Pin | MCU Pin | Description |
|-------------|---------|-------------|
| VDD         | 3.3V    | Power supply (2.5V - 3.6V) |
| GND         | GND     | Ground |
| SPC         | SCK     | SPI clock |
| SDI         | MOSI    | SPI data in |
| SDO         | MISO    | SPI data out |
| CS          | CS      | Chip select (active low) |

**SPI Mode**: Mode 0 (CPOL=0, CPHA=0) or Mode 3 (CPOL=1, CPHA=1)
**Max SPI Clock**: 10 MHz

## Magnetic Field Sensitivity

The LIS2MDL has a fixed sensitivity of **1.5 mgauss/LSB** (0.0015 gauss/LSB or 0.15 μT/LSB).

- Full scale range: ±50 gauss
- 16-bit output: -32768 to +32767 LSB
- Maximum measurable field: ±49.152 gauss

## Examples

See the [examples](examples/) directory for complete examples:

- [`blocking_i2c.rs`](examples/blocking_i2c.rs) - Blocking I2C example
- [`embassy_stm32_i2c.rs`](examples/embassy_stm32_i2c.rs) - Async I2C with Embassy
- [`embassy_stm32_spi.rs`](examples/embassy_stm32_spi.rs) - Async SPI with Embassy
- [`calibration.rs`](examples/calibration.rs) - Hard-iron calibration procedure

## Minimum Supported Rust Version (MSRV)

This crate requires Rust 1.75 or later.

## Architecture

This driver is built using the [`device-driver`](https://crates.io/crates/device-driver) framework, which generates type-safe register access code from a YAML manifest. This approach ensures:

- Zero-cost abstractions
- Compile-time register field validation
- Automatic documentation from register descriptions
- Consistent API across different sensors

## References

- [LIS2MDL Datasheet](https://www.st.com/resource/en/datasheet/lis2mdl.pdf)
- [LIS2MDL Application Note AN5069](https://www.st.com/resource/en/application_note/dm00393911.pdf)

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
