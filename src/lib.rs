//! High-level LIS2MDL magnetometer driver with synchronous and asynchronous support.
//!
//! This driver supports both I2C and SPI communication interfaces, with blocking and async modes.
//!
//! # Examples
//!
//! ## Blocking I2C
//!
//! ```no_run
//! use lis2mdl::Lis2mdl;
//! # use embedded_hal::i2c::I2c;
//! # fn example<I2C: I2c>(i2c: I2C) -> Result<(), lis2mdl::Error<I2C::Error>>
//! # where I2C::Error: core::fmt::Debug {
//!
//! // Create driver with default configuration
//! let mut sensor = Lis2mdl::new_i2c(i2c)?;
//!
//! // Read magnetic field in gauss (floating point)
//! let mag = sensor.read_gauss()?;
//! # Ok(())
//! # }
//! ```
//!
//! ## Blocking SPI
//!
//! ```no_run
//! use lis2mdl::Lis2mdl;
//! # use embedded_hal::spi::SpiDevice;
//! # fn example<SPI: SpiDevice>(spi: SPI) -> Result<(), lis2mdl::Error<SPI::Error>>
//! # where SPI::Error: core::fmt::Debug {
//!
//! // Create driver with SPI interface
//! let mut sensor = Lis2mdl::new_spi(spi)?;
//!
//! // Read magnetic field in milligauss (integer)
//! let mag_mg = sensor.read_mgauss()?;
//! # Ok(())
//! # }
//! ```
//!
//! ## Async I2C
//!
//! ```no_run
//! use lis2mdl::Lis2mdlAsync;
//! # use embedded_hal_async::i2c::I2c;
//! # async fn example<I2C: I2c>(i2c: I2C) -> Result<(), lis2mdl::Error<I2C::Error>>
//! # where I2C::Error: core::fmt::Debug {
//!
//! // Create async driver with I2C
//! let mut sensor = Lis2mdlAsync::new_i2c(i2c).await?;
//!
//! // Read magnetic field asynchronously
//! let mag = sensor.read_gauss().await?;
//! # Ok(())
//! # }
//! ```
//!
//! ## Async SPI
//!
//! ```no_run
//! use lis2mdl::Lis2mdlAsync;
//! # use embedded_hal_async::spi::SpiDevice;
//! # async fn example<SPI: SpiDevice>(spi: SPI) -> Result<(), lis2mdl::Error<SPI::Error>>
//! # where SPI::Error: core::fmt::Debug {
//!
//! // Create async driver with SPI
//! let mut sensor = Lis2mdlAsync::new_spi(spi).await?;
//!
//! // Read magnetic field asynchronously
//! let mag = sensor.read_gauss().await?;
//! # Ok(())
//! # }
//! ```
//!
//! ## Custom Configuration
//!
//! ```no_run
//! use lis2mdl::{Lis2mdl, Lis2mdlConfig, Odr, OperatingMode, OffsetCancellation};
//! # use embedded_hal::i2c::I2c;
//! # fn example<I2C: I2c>(i2c: I2C) -> Result<(), lis2mdl::Error<I2C::Error>>
//! # where I2C::Error: core::fmt::Debug {
//!
//! let config = Lis2mdlConfig {
//!     odr: Odr::Hz100,                                      // 100 Hz output data rate
//!     mode: OperatingMode::LowPower,                       // Low-power mode
//!     offset_cancellation: OffsetCancellation::Enabled,    // Enable offset cancellation
//!     temperature_compensation: true,                      // Enable temperature compensation
//!     ..Default::default()
//! };
//!
//! let mut sensor = Lis2mdl::new_i2c_with_config(i2c, config)?;
//! # Ok(())
//! # }
//! ```
//!
//! ## Hard-Iron Offset Calibration
//!
//! ```no_run
//! use lis2mdl::Lis2mdl;
//! # use embedded_hal::i2c::I2c;
//! # fn example<I2C: I2c>(i2c: I2C) -> Result<(), lis2mdl::Error<I2C::Error>>
//! # where I2C::Error: core::fmt::Debug {
//!
//! let mut sensor = Lis2mdl::new_i2c(i2c)?;
//!
//! // Set hard-iron offset calibration values
//! sensor.set_offset(100, -50, 75)?;
//!
//! // Read current offset values
//! let (x, y, z) = sensor.get_offset()?;
//! # Ok(())
//! # }
//! ```
//!
//! # SPI Mode
//!
//! The LIS2MDL requires SPI Mode 0 or Mode 3 (CPOL=0, CPHA=0 or CPOL=1, CPHA=1).
//! The driver uses the `embedded-hal` `SpiDevice` trait which handles chip select automatically.

#![no_std]
#![deny(missing_docs)]
#![deny(warnings)]
#![allow(clippy::missing_errors_doc)]

use core::fmt::Debug;

#[cfg(feature = "async")]
use device_driver::{AsyncBufferInterface, AsyncRegisterInterface};
use device_driver::{BufferInterface, BufferInterfaceError, RegisterInterface};
use embedded_hal as hal;
#[cfg(feature = "async")]
use embedded_hal_async as hal_async;
use hal::i2c::I2c;
#[cfg(feature = "async")]
use hal_async::i2c::I2c as AsyncI2c;
#[cfg(feature = "async")]
use hal_async::spi::SpiDevice as AsyncSpiDevice;

#[allow(unsafe_code)]
#[allow(missing_docs)]
#[allow(clippy::doc_markdown, clippy::missing_errors_doc, clippy::identity_op)]
mod generated {
    device_driver::create_device!(
        device_name: Lis2mdlDevice,
        manifest: "src/lis2mdl.yaml"
    );
}

pub use generated::{field_sets, Lis2mdlDevice, Md, Odr};

/// Fixed I2C address for the LIS2MDL.
pub const I2C_ADDRESS: u8 = 0x1E;

/// Device ID value expected in WHO_AM_I register.
pub const DEVICE_ID: u8 = 0x40;

/// Sensitivity in milligauss per LSB (1.5 mgauss/LSB).
pub const SENSITIVITY_MGAUSS_PER_LSB: f32 = 1.5;

/// Sensitivity in gauss per LSB.
pub const SENSITIVITY_GAUSS_PER_LSB: f32 = SENSITIVITY_MGAUSS_PER_LSB / 1000.0;

/// Sensor operating mode controlling resolution and power consumption.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum OperatingMode {
    /// High-resolution mode (default).
    HighResolution,
    /// Low-power mode.
    LowPower,
}

impl OperatingMode {
    const fn lp(self) -> bool {
        matches!(self, OperatingMode::LowPower)
    }
}

/// Offset cancellation mode.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum OffsetCancellation {
    /// Offset cancellation disabled.
    Disabled,
    /// Offset cancellation enabled (continuous mode).
    Enabled,
    /// Offset cancellation in single measurement mode.
    SingleMeasurement,
}

/// High-level configuration applied during initialisation or runtime updates.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Lis2mdlConfig {
    /// Output data rate.
    pub odr: Odr,
    /// Operating mode (resolution/power trade-off).
    pub mode: OperatingMode,
    /// Measurement mode (continuous or single-shot).
    pub measurement_mode: Md,
    /// Enable block data updates.
    pub block_data_update: bool,
    /// Enable temperature compensation.
    pub temperature_compensation: bool,
    /// Offset cancellation mode.
    pub offset_cancellation: OffsetCancellation,
    /// Enable low-pass filter (bandwidth ODR/4, disabled = ODR/2).
    pub low_pass_filter: bool,
}

impl Default for Lis2mdlConfig {
    fn default() -> Self {
        Self {
            odr: Odr::Hz10,
            mode: OperatingMode::HighResolution,
            measurement_mode: Md::Continuous,
            block_data_update: true,
            temperature_compensation: true,
            offset_cancellation: OffsetCancellation::Disabled,
            low_pass_filter: false,
        }
    }
}

impl Lis2mdlConfig {
    fn sensitivity_gauss_per_lsb(self) -> f32 {
        SENSITIVITY_GAUSS_PER_LSB
    }

    fn sensitivity_mgauss_per_lsb(self) -> f32 {
        SENSITIVITY_MGAUSS_PER_LSB
    }
}

/// Magnetic field measurement in generic units.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct MagneticField<T> {
    /// X-axis magnetic field.
    pub x: T,
    /// Y-axis magnetic field.
    pub y: T,
    /// Z-axis magnetic field.
    pub z: T,
}

impl<T> MagneticField<T> {
    /// Create a new magnetic field measurement.
    pub const fn new(x: T, y: T, z: T) -> Self {
        Self { x, y, z }
    }
}

/// Temperature measurement in degrees Celsius.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Temperature {
    /// Temperature in degrees Celsius.
    pub celsius: f32,
}

impl Temperature {
    /// Create a new temperature measurement.
    pub const fn new(celsius: f32) -> Self {
        Self { celsius }
    }
}

/// Parsed status register snapshot.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[must_use]
pub struct Status {
    /// X-axis new data available.
    pub x_ready: bool,
    /// Y-axis new data available.
    pub y_ready: bool,
    /// Z-axis new data available.
    pub z_ready: bool,
    /// All axes new data available.
    pub xyz_ready: bool,
    /// X-axis data overrun.
    pub x_overrun: bool,
    /// Y-axis data overrun.
    pub y_overrun: bool,
    /// Z-axis data overrun.
    pub z_overrun: bool,
    /// Any axis data overrun.
    pub xyz_overrun: bool,
}

impl Status {
    /// Create a status snapshot from the raw register value.
    #[must_use]
    pub fn from_raw(raw: field_sets::StatusReg) -> Self {
        Self {
            x_ready: raw.xda(),
            y_ready: raw.yda(),
            z_ready: raw.zda(),
            xyz_ready: raw.zyxda(),
            x_overrun: raw.xor(),
            y_overrun: raw.yor(),
            z_overrun: raw.zor(),
            xyz_overrun: raw.zyxor(),
        }
    }

    /// Returns true if new data is available on all axes.
    #[must_use]
    pub const fn is_ready(self) -> bool {
        self.xyz_ready
    }

    /// Returns true if data overrun occurred on any axis.
    #[must_use]
    pub const fn has_overrun(self) -> bool {
        self.xyz_overrun
    }
}

impl From<field_sets::StatusReg> for Status {
    fn from(raw: field_sets::StatusReg) -> Self {
        Self::from_raw(raw)
    }
}

/// Driver error type.
#[derive(Debug)]
pub enum Error<E> {
    /// Communication error.
    Interface(E),
    /// Invalid device ID.
    InvalidDevice,
    /// Invalid parameter or configuration.
    InvalidParameter,
}

impl<E> From<E> for Error<E> {
    fn from(error: E) -> Self {
        Error::Interface(error)
    }
}

/// Blocking I2C interface wrapper.
pub struct DeviceInterface<I2C> {
    /// Underlying I2C bus.
    pub i2c: I2C,
    /// Slave address.
    pub address: u8,
}

/// Asynchronous I2C interface wrapper.
#[cfg(feature = "async")]
pub struct DeviceInterfaceAsync<I2C> {
    /// Underlying async I2C bus.
    pub i2c: I2C,
    /// Slave address.
    pub address: u8,
}

/// Blocking SPI interface wrapper.
pub struct SpiInterface<SPI> {
    /// Underlying SPI bus (with CS management).
    pub spi: SPI,
}

/// Asynchronous SPI interface wrapper.
#[cfg(feature = "async")]
pub struct SpiInterfaceAsync<SPI> {
    /// Underlying async SPI bus (with CS management).
    pub spi: SPI,
}

impl<I2C> BufferInterfaceError for DeviceInterface<I2C>
where
    I2C: hal::i2c::I2c,
{
    type Error = I2C::Error;
}

#[cfg(feature = "async")]
impl<I2C> BufferInterfaceError for DeviceInterfaceAsync<I2C>
where
    I2C: AsyncI2c,
{
    type Error = I2C::Error;
}

impl<I2C> RegisterInterface for DeviceInterface<I2C>
where
    I2C: hal::i2c::I2c,
{
    type Error = I2C::Error;
    type AddressType = u8;

    fn write_register(&mut self, address: Self::AddressType, _size_bits: u32, data: &[u8]) -> Result<(), Self::Error> {
        let mut buf = [0u8; 1 + 8];
        buf[0] = address;
        let end = 1 + data.len();
        buf[1..end].copy_from_slice(data);
        self.i2c.write(self.address, &buf[..end])
    }

    fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address, &[address], data)
    }
}

#[cfg(feature = "async")]
impl<I2C> AsyncRegisterInterface for DeviceInterfaceAsync<I2C>
where
    I2C: AsyncI2c,
{
    type Error = I2C::Error;
    type AddressType = u8;

    async fn write_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        let mut buf = [0u8; 1 + 8];
        buf[0] = address;
        let end = 1 + data.len();
        buf[1..end].copy_from_slice(data);
        self.i2c.write(self.address, &buf[..end]).await
    }

    async fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address, &[address], data).await
    }
}

impl<I2C> BufferInterface for DeviceInterface<I2C>
where
    I2C: hal::i2c::I2c,
{
    type AddressType = u8;

    fn read(
        &mut self,
        address: Self::AddressType,
        buf: &mut [u8],
    ) -> Result<usize, <Self as RegisterInterface>::Error> {
        self.i2c.write_read(self.address, &[address], buf)?;
        Ok(buf.len())
    }

    fn write(&mut self, address: Self::AddressType, buf: &[u8]) -> Result<usize, <Self as RegisterInterface>::Error> {
        let mut data = [0u8; 1 + 32];
        data[0] = address;
        let end = 1 + buf.len();
        data[1..end].copy_from_slice(buf);
        self.i2c.write(self.address, &data[..end])?;
        Ok(buf.len())
    }

    fn flush(&mut self, _address: Self::AddressType) -> Result<(), <Self as RegisterInterface>::Error> {
        Ok(())
    }
}

#[cfg(feature = "async")]
impl<I2C> AsyncBufferInterface for DeviceInterfaceAsync<I2C>
where
    I2C: AsyncI2c,
{
    type AddressType = u8;

    async fn read(&mut self, address: Self::AddressType, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.i2c.write_read(self.address, &[address], buf).await?;
        Ok(buf.len())
    }

    async fn write(&mut self, address: Self::AddressType, buf: &[u8]) -> Result<usize, Self::Error> {
        let mut data = [0u8; 1 + 32];
        data[0] = address;
        let end = 1 + buf.len();
        data[1..end].copy_from_slice(buf);
        self.i2c.write(self.address, &data[..end]).await?;
        Ok(buf.len())
    }

    async fn flush(&mut self, _address: Self::AddressType) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl<SPI> BufferInterfaceError for SpiInterface<SPI>
where
    SPI: hal::spi::SpiDevice,
{
    type Error = SPI::Error;
}

#[cfg(feature = "async")]
impl<SPI> BufferInterfaceError for SpiInterfaceAsync<SPI>
where
    SPI: hal_async::spi::SpiDevice,
{
    type Error = SPI::Error;
}

impl<SPI> RegisterInterface for SpiInterface<SPI>
where
    SPI: hal::spi::SpiDevice,
{
    type Error = SPI::Error;
    type AddressType = u8;

    fn write_register(&mut self, address: Self::AddressType, _size_bits: u32, data: &[u8]) -> Result<(), Self::Error> {
        let mut buf = [0u8; 1 + 8];
        buf[0] = address; // Write: bit 7 = 0
        let end = 1 + data.len();
        buf[1..end].copy_from_slice(data);
        self.spi.write(&buf[..end])
    }

    fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        let addr_byte = 0x80 | address; // Read: bit 7 = 1
        self.spi.transaction(&mut [
            hal::spi::Operation::Write(&[addr_byte]),
            hal::spi::Operation::Read(data),
        ])
    }
}

#[cfg(feature = "async")]
impl<SPI> AsyncRegisterInterface for SpiInterfaceAsync<SPI>
where
    SPI: hal_async::spi::SpiDevice,
{
    type Error = SPI::Error;
    type AddressType = u8;

    async fn write_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        let mut buf = [0u8; 1 + 8];
        buf[0] = address; // Write: bit 7 = 0
        let end = 1 + data.len();
        buf[1..end].copy_from_slice(data);
        self.spi.write(&buf[..end]).await
    }

    async fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        let addr_byte = 0x80 | address; // Read: bit 7 = 1
        self.spi.transaction(&mut [
            hal_async::spi::Operation::Write(&[addr_byte]),
            hal_async::spi::Operation::Read(data),
        ]).await
    }
}

impl<SPI> BufferInterface for SpiInterface<SPI>
where
    SPI: hal::spi::SpiDevice,
{
    type AddressType = u8;

    fn read(
        &mut self,
        address: Self::AddressType,
        buf: &mut [u8],
    ) -> Result<usize, <Self as RegisterInterface>::Error> {
        // Multi-byte read: bit 7 = 1 (read), bit 6 = 1 (auto-increment)
        let addr_byte = 0xC0 | address;
        self.spi.transaction(&mut [
            hal::spi::Operation::Write(&[addr_byte]),
            hal::spi::Operation::Read(buf),
        ])?;
        Ok(buf.len())
    }

    fn write(&mut self, address: Self::AddressType, buf: &[u8]) -> Result<usize, <Self as RegisterInterface>::Error> {
        let mut data = [0u8; 1 + 32];
        data[0] = 0x40 | address; // Write with auto-increment: bit 6 = 1
        let end = 1 + buf.len();
        data[1..end].copy_from_slice(buf);
        self.spi.write(&data[..end])?;
        Ok(buf.len())
    }

    fn flush(&mut self, _address: Self::AddressType) -> Result<(), <Self as RegisterInterface>::Error> {
        Ok(())
    }
}

#[cfg(feature = "async")]
impl<SPI> AsyncBufferInterface for SpiInterfaceAsync<SPI>
where
    SPI: hal_async::spi::SpiDevice,
{
    type AddressType = u8;

    async fn read(&mut self, address: Self::AddressType, buf: &mut [u8]) -> Result<usize, Self::Error> {
        // Multi-byte read: bit 7 = 1 (read), bit 6 = 1 (auto-increment)
        let addr_byte = 0xC0 | address;
        self.spi.transaction(&mut [
            hal_async::spi::Operation::Write(&[addr_byte]),
            hal_async::spi::Operation::Read(buf),
        ]).await?;
        Ok(buf.len())
    }

    async fn write(&mut self, address: Self::AddressType, buf: &[u8]) -> Result<usize, Self::Error> {
        let mut data = [0u8; 1 + 32];
        data[0] = 0x40 | address; // Write with auto-increment: bit 6 = 1
        let end = 1 + buf.len();
        data[1..end].copy_from_slice(buf);
        self.spi.write(&data[..end]).await?;
        Ok(buf.len())
    }

    async fn flush(&mut self, _address: Self::AddressType) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// Blocking LIS2MDL driver.
pub struct Lis2mdl<IFACE> {
    device: Lis2mdlDevice<IFACE>,
    config: Lis2mdlConfig,
}

/// Type alias for I2C-based blocking driver.
pub type Lis2mdlI2c<I2C> = Lis2mdl<DeviceInterface<I2C>>;

/// Type alias for SPI-based blocking driver.
pub type Lis2mdlSpi<SPI> = Lis2mdl<SpiInterface<SPI>>;

// Generic implementation for all interface types
impl<IFACE> Lis2mdl<IFACE>
where
    IFACE: RegisterInterface<AddressType = u8, Error = <IFACE as BufferInterfaceError>::Error> + BufferInterface<AddressType = u8>,
    <IFACE as RegisterInterface>::Error: Debug,
{
    /// Return the active configuration.
    pub const fn config(&self) -> Lis2mdlConfig {
        self.config
    }

    /// Update the sensor configuration.
    pub fn set_config(&mut self, config: Lis2mdlConfig) -> Result<(), Error<<IFACE as RegisterInterface>::Error>> {
        self.apply_config(config)?;
        self.config = config;
        Ok(())
    }

    /// Read raw magnetic field sample triplet (16-bit signed values).
    pub fn read_raw(&mut self) -> Result<MagneticField<i16>, Error<<IFACE as RegisterInterface>::Error>> {
        let mut buf = [0u8; 6];
        let mut mag_data = self.device.mag_data_read();
        let mut offset = 0;
        while offset < buf.len() {
            let read = mag_data.read(&mut buf[offset..]).map_err(Error::from)?;
            if read == 0 {
                return Err(Error::InvalidParameter);
            }
            offset += read;
        }
        Ok(decode_raw(&buf))
    }

    /// Read magnetic field expressed in milligauss (integer).
    pub fn read_mgauss(&mut self) -> Result<MagneticField<i16>, Error<<IFACE as RegisterInterface>::Error>> {
        let raw = self.read_raw()?;
        Ok(scale_to_mgauss(raw, self.config.sensitivity_mgauss_per_lsb()))
    }

    /// Read magnetic field expressed in gauss (floating point).
    pub fn read_gauss(&mut self) -> Result<MagneticField<f32>, Error<<IFACE as RegisterInterface>::Error>> {
        let raw = self.read_raw()?;
        Ok(scale_to_gauss(raw, self.config.sensitivity_gauss_per_lsb()))
    }

    /// Read temperature sensor output.
    pub fn read_temperature(&mut self) -> Result<Temperature, Error<<IFACE as RegisterInterface>::Error>> {
        let temp_l = self.device.temp_out_l_reg().read().map_err(Error::from)?;
        let temp_h = self.device.temp_out_h_reg().read().map_err(Error::from)?;
        let temp_l_bytes: [u8; 1] = temp_l.into();
        let temp_h_bytes: [u8; 1] = temp_h.into();
        let temp_raw = i16::from_le_bytes([temp_l_bytes[0], temp_h_bytes[0]]);
        // Temperature sensitivity is 8 LSB/°C with 25°C offset
        let celsius = (temp_raw as f32 / 8.0) + 25.0;
        Ok(Temperature::new(celsius))
    }

    /// Read the status register.
    pub fn status(&mut self) -> Result<Status, Error<<IFACE as RegisterInterface>::Error>> {
        let status = self.device.status_reg().read().map_err(Error::from)?;
        Ok(Status::from(status))
    }

    /// Set hard-iron offset calibration values (16-bit signed LSB units).
    pub fn set_offset(&mut self, x: i16, y: i16, z: i16) -> Result<(), Error<<IFACE as RegisterInterface>::Error>> {
        let x_bytes = x.to_le_bytes();
        let y_bytes = y.to_le_bytes();
        let z_bytes = z.to_le_bytes();

        self.device.offset_x_reg_l().write(|r| *r = [x_bytes[0]].into()).map_err(Error::from)?;
        self.device.offset_x_reg_h().write(|r| *r = [x_bytes[1]].into()).map_err(Error::from)?;
        self.device.offset_y_reg_l().write(|r| *r = [y_bytes[0]].into()).map_err(Error::from)?;
        self.device.offset_y_reg_h().write(|r| *r = [y_bytes[1]].into()).map_err(Error::from)?;
        self.device.offset_z_reg_l().write(|r| *r = [z_bytes[0]].into()).map_err(Error::from)?;
        self.device.offset_z_reg_h().write(|r| *r = [z_bytes[1]].into()).map_err(Error::from)?;

        Ok(())
    }

    /// Get current hard-iron offset calibration values.
    pub fn get_offset(&mut self) -> Result<(i16, i16, i16), Error<<IFACE as RegisterInterface>::Error>> {
        let x_l = self.device.offset_x_reg_l().read().map_err(Error::from)?;
        let x_h = self.device.offset_x_reg_h().read().map_err(Error::from)?;
        let y_l = self.device.offset_y_reg_l().read().map_err(Error::from)?;
        let y_h = self.device.offset_y_reg_h().read().map_err(Error::from)?;
        let z_l = self.device.offset_z_reg_l().read().map_err(Error::from)?;
        let z_h = self.device.offset_z_reg_h().read().map_err(Error::from)?;

        let x_l_bytes: [u8; 1] = x_l.into();
        let x_h_bytes: [u8; 1] = x_h.into();
        let y_l_bytes: [u8; 1] = y_l.into();
        let y_h_bytes: [u8; 1] = y_h.into();
        let z_l_bytes: [u8; 1] = z_l.into();
        let z_h_bytes: [u8; 1] = z_h.into();

        let x = i16::from_le_bytes([x_l_bytes[0], x_h_bytes[0]]);
        let y = i16::from_le_bytes([y_l_bytes[0], y_h_bytes[0]]);
        let z = i16::from_le_bytes([z_l_bytes[0], z_h_bytes[0]]);

        Ok((x, y, z))
    }

    /// Trigger a soft reset.
    pub fn soft_reset(&mut self) -> Result<(), Error<<IFACE as RegisterInterface>::Error>> {
        self.device
            .cfg_reg_a()
            .write(|reg: &mut field_sets::CfgRegA| {
                reg.set_soft_rst(true);
            })
            .map_err(Error::from)
    }

    /// Issue a reboot command to reload memory content.
    pub fn reboot(&mut self) -> Result<(), Error<<IFACE as RegisterInterface>::Error>> {
        self.device
            .cfg_reg_a()
            .write(|reg: &mut field_sets::CfgRegA| {
                reg.set_reboot(true);
            })
            .map_err(Error::from)
    }

    /// Trigger a single measurement (only in single-shot mode).
    pub fn trigger_measurement(&mut self) -> Result<(), Error<<IFACE as RegisterInterface>::Error>> {
        self.device
            .cfg_reg_a()
            .modify(|reg: &mut field_sets::CfgRegA| {
                reg.set_md(Md::Single);
            })
            .map_err(Error::from)
    }

    fn apply_config(&mut self, config: Lis2mdlConfig) -> Result<(), Error<<IFACE as RegisterInterface>::Error>> {
        // Configure register A
        self.device
            .cfg_reg_a()
            .write(|reg: &mut field_sets::CfgRegA| {
                reg.set_md(config.measurement_mode);
                reg.set_odr(config.odr);
                reg.set_lp(config.mode.lp());
                reg.set_comp_temp_en(config.temperature_compensation);
            })
            .map_err(Error::from)?;

        // Configure register B
        self.device
            .cfg_reg_b()
            .write(|reg: &mut field_sets::CfgRegB| {
                reg.set_lpf(config.low_pass_filter);
                match config.offset_cancellation {
                    OffsetCancellation::Disabled => {
                        reg.set_off_canc(false);
                        reg.set_off_canc_one_shot(false);
                    }
                    OffsetCancellation::Enabled => {
                        reg.set_off_canc(true);
                        reg.set_off_canc_one_shot(false);
                    }
                    OffsetCancellation::SingleMeasurement => {
                        reg.set_off_canc(false);
                        reg.set_off_canc_one_shot(true);
                    }
                }
            })
            .map_err(Error::from)?;

        // Configure register C
        self.device
            .cfg_reg_c()
            .write(|reg: &mut field_sets::CfgRegC| {
                reg.set_bdu(config.block_data_update);
            })
            .map_err(Error::from)?;

        Ok(())
    }
}

// I2C-specific constructors and methods
impl<I2C> Lis2mdl<DeviceInterface<I2C>>
where
    I2C: I2c,
    I2C::Error: Debug,
{
    /// Create a new I2C driver with the default configuration.
    pub fn new_i2c(i2c: I2C) -> Result<Self, Error<I2C::Error>> {
        Self::new_i2c_with_config(i2c, Lis2mdlConfig::default())
    }

    /// Create a new I2C driver with an explicit configuration.
    pub fn new_i2c_with_config(i2c: I2C, config: Lis2mdlConfig) -> Result<Self, Error<I2C::Error>> {
        let interface = DeviceInterface {
            i2c,
            address: I2C_ADDRESS,
        };
        let mut device = Lis2mdlDevice::new(interface);
        verify_device_id(&mut device)?;
        let mut this = Self { device, config };
        this.apply_config(config)?;
        Ok(this)
    }

    /// Access the generated register API directly.
    pub fn device(&mut self) -> &mut Lis2mdlDevice<DeviceInterface<I2C>> {
        &mut self.device
    }

    /// Consume the driver and return the underlying I2C bus.
    pub fn destroy(self) -> I2C {
        self.device.interface.i2c
    }
}

// SPI-specific constructors and methods
impl<SPI> Lis2mdl<SpiInterface<SPI>>
where
    SPI: hal::spi::SpiDevice,
    SPI::Error: Debug,
{
    /// Create a new SPI driver with the default configuration.
    pub fn new_spi(spi: SPI) -> Result<Self, Error<SPI::Error>> {
        Self::new_spi_with_config(spi, Lis2mdlConfig::default())
    }

    /// Create a new SPI driver with an explicit configuration.
    pub fn new_spi_with_config(spi: SPI, config: Lis2mdlConfig) -> Result<Self, Error<SPI::Error>> {
        let interface = SpiInterface { spi };
        let mut device = Lis2mdlDevice::new(interface);
        verify_device_id(&mut device)?;
        let mut this = Self { device, config };
        this.apply_config(config)?;
        Ok(this)
    }

    /// Access the generated register API directly.
    pub fn device(&mut self) -> &mut Lis2mdlDevice<SpiInterface<SPI>> {
        &mut self.device
    }

    /// Consume the driver and return the underlying SPI bus.
    pub fn destroy(self) -> SPI {
        self.device.interface.spi
    }
}

/// Asynchronous LIS2MDL driver.
#[cfg(feature = "async")]
pub struct Lis2mdlAsync<IFACE> {
    device: Lis2mdlDevice<IFACE>,
    config: Lis2mdlConfig,
}

/// Type alias for I2C-based async driver.
#[cfg(feature = "async")]
pub type Lis2mdlI2cAsync<I2C> = Lis2mdlAsync<DeviceInterfaceAsync<I2C>>;

/// Type alias for SPI-based async driver.
#[cfg(feature = "async")]
pub type Lis2mdlSpiAsync<SPI> = Lis2mdlAsync<SpiInterfaceAsync<SPI>>;

#[cfg(feature = "async")]
// Generic implementation for all async interface types
impl<IFACE> Lis2mdlAsync<IFACE>
where
    IFACE: AsyncRegisterInterface<AddressType = u8, Error = <IFACE as BufferInterfaceError>::Error> + AsyncBufferInterface<AddressType = u8>,
    <IFACE as AsyncRegisterInterface>::Error: Debug,
{
    /// Return the active configuration.
    pub const fn config(&self) -> Lis2mdlConfig {
        self.config
    }

    /// Update the sensor configuration asynchronously.
    pub async fn set_config(&mut self, config: Lis2mdlConfig) -> Result<(), Error<<IFACE as AsyncRegisterInterface>::Error>> {
        self.apply_config(config).await?;
        self.config = config;
        Ok(())
    }

    /// Read raw magnetic field sample triplet asynchronously.
    pub async fn read_raw(&mut self) -> Result<MagneticField<i16>, Error<<IFACE as AsyncRegisterInterface>::Error>> {
        let mut buf = [0u8; 6];
        let mut mag_data = self.device.mag_data_read();
        let mut offset = 0;
        while offset < buf.len() {
            let read = mag_data.read_async(&mut buf[offset..]).await.map_err(Error::from)?;
            if read == 0 {
                return Err(Error::InvalidParameter);
            }
            offset += read;
        }
        Ok(decode_raw(&buf))
    }

    /// Read magnetic field expressed in milligauss asynchronously.
    pub async fn read_mgauss(&mut self) -> Result<MagneticField<i16>, Error<<IFACE as AsyncRegisterInterface>::Error>> {
        let raw = self.read_raw().await?;
        Ok(scale_to_mgauss(raw, self.config.sensitivity_mgauss_per_lsb()))
    }

    /// Read magnetic field expressed in gauss asynchronously.
    pub async fn read_gauss(&mut self) -> Result<MagneticField<f32>, Error<<IFACE as AsyncRegisterInterface>::Error>> {
        let raw = self.read_raw().await?;
        Ok(scale_to_gauss(raw, self.config.sensitivity_gauss_per_lsb()))
    }

    /// Read temperature sensor output asynchronously.
    pub async fn read_temperature(&mut self) -> Result<Temperature, Error<<IFACE as AsyncRegisterInterface>::Error>> {
        let temp_l = self.device.temp_out_l_reg().read_async().await.map_err(Error::from)?;
        let temp_h = self.device.temp_out_h_reg().read_async().await.map_err(Error::from)?;
        let temp_l_bytes: [u8; 1] = temp_l.into();
        let temp_h_bytes: [u8; 1] = temp_h.into();
        let temp_raw = i16::from_le_bytes([temp_l_bytes[0], temp_h_bytes[0]]);
        let celsius = (temp_raw as f32 / 8.0) + 25.0;
        Ok(Temperature::new(celsius))
    }

    /// Read the status register asynchronously.
    pub async fn status(&mut self) -> Result<Status, Error<<IFACE as AsyncRegisterInterface>::Error>> {
        let status = self.device.status_reg().read_async().await.map_err(Error::from)?;
        Ok(Status::from(status))
    }

    /// Set hard-iron offset calibration values asynchronously.
    pub async fn set_offset(&mut self, x: i16, y: i16, z: i16) -> Result<(), Error<<IFACE as AsyncRegisterInterface>::Error>> {
        let x_bytes = x.to_le_bytes();
        let y_bytes = y.to_le_bytes();
        let z_bytes = z.to_le_bytes();

        self.device.offset_x_reg_l().write_async(|r| *r = [x_bytes[0]].into()).await.map_err(Error::from)?;
        self.device.offset_x_reg_h().write_async(|r| *r = [x_bytes[1]].into()).await.map_err(Error::from)?;
        self.device.offset_y_reg_l().write_async(|r| *r = [y_bytes[0]].into()).await.map_err(Error::from)?;
        self.device.offset_y_reg_h().write_async(|r| *r = [y_bytes[1]].into()).await.map_err(Error::from)?;
        self.device.offset_z_reg_l().write_async(|r| *r = [z_bytes[0]].into()).await.map_err(Error::from)?;
        self.device.offset_z_reg_h().write_async(|r| *r = [z_bytes[1]].into()).await.map_err(Error::from)?;

        Ok(())
    }

    /// Get current hard-iron offset calibration values asynchronously.
    pub async fn get_offset(&mut self) -> Result<(i16, i16, i16), Error<<IFACE as AsyncRegisterInterface>::Error>> {
        let x_l = self.device.offset_x_reg_l().read_async().await.map_err(Error::from)?;
        let x_h = self.device.offset_x_reg_h().read_async().await.map_err(Error::from)?;
        let y_l = self.device.offset_y_reg_l().read_async().await.map_err(Error::from)?;
        let y_h = self.device.offset_y_reg_h().read_async().await.map_err(Error::from)?;
        let z_l = self.device.offset_z_reg_l().read_async().await.map_err(Error::from)?;
        let z_h = self.device.offset_z_reg_h().read_async().await.map_err(Error::from)?;

        let x_l_bytes: [u8; 1] = x_l.into();
        let x_h_bytes: [u8; 1] = x_h.into();
        let y_l_bytes: [u8; 1] = y_l.into();
        let y_h_bytes: [u8; 1] = y_h.into();
        let z_l_bytes: [u8; 1] = z_l.into();
        let z_h_bytes: [u8; 1] = z_h.into();

        let x = i16::from_le_bytes([x_l_bytes[0], x_h_bytes[0]]);
        let y = i16::from_le_bytes([y_l_bytes[0], y_h_bytes[0]]);
        let z = i16::from_le_bytes([z_l_bytes[0], z_h_bytes[0]]);

        Ok((x, y, z))
    }

    /// Trigger a soft reset asynchronously.
    pub async fn soft_reset(&mut self) -> Result<(), Error<<IFACE as AsyncRegisterInterface>::Error>> {
        self.device
            .cfg_reg_a()
            .write_async(|reg: &mut field_sets::CfgRegA| {
                reg.set_soft_rst(true);
            })
            .await
            .map_err(Error::from)
    }

    /// Issue a reboot command asynchronously.
    pub async fn reboot(&mut self) -> Result<(), Error<<IFACE as AsyncRegisterInterface>::Error>> {
        self.device
            .cfg_reg_a()
            .write_async(|reg: &mut field_sets::CfgRegA| {
                reg.set_reboot(true);
            })
            .await
            .map_err(Error::from)
    }

    /// Trigger a single measurement asynchronously.
    pub async fn trigger_measurement(&mut self) -> Result<(), Error<<IFACE as AsyncRegisterInterface>::Error>> {
        self.device
            .cfg_reg_a()
            .modify_async(|reg: &mut field_sets::CfgRegA| {
                reg.set_md(Md::Single);
            })
            .await
            .map_err(Error::from)
    }

    async fn apply_config(&mut self, config: Lis2mdlConfig) -> Result<(), Error<<IFACE as AsyncRegisterInterface>::Error>> {
        // Configure register A
        self.device
            .cfg_reg_a()
            .write_async(|reg: &mut field_sets::CfgRegA| {
                reg.set_md(config.measurement_mode);
                reg.set_odr(config.odr);
                reg.set_lp(config.mode.lp());
                reg.set_comp_temp_en(config.temperature_compensation);
            })
            .await
            .map_err(Error::from)?;

        // Configure register B
        self.device
            .cfg_reg_b()
            .write_async(|reg: &mut field_sets::CfgRegB| {
                reg.set_lpf(config.low_pass_filter);
                match config.offset_cancellation {
                    OffsetCancellation::Disabled => {
                        reg.set_off_canc(false);
                        reg.set_off_canc_one_shot(false);
                    }
                    OffsetCancellation::Enabled => {
                        reg.set_off_canc(true);
                        reg.set_off_canc_one_shot(false);
                    }
                    OffsetCancellation::SingleMeasurement => {
                        reg.set_off_canc(false);
                        reg.set_off_canc_one_shot(true);
                    }
                }
            })
            .await
            .map_err(Error::from)?;

        // Configure register C
        self.device
            .cfg_reg_c()
            .write_async(|reg: &mut field_sets::CfgRegC| {
                reg.set_bdu(config.block_data_update);
            })
            .await
            .map_err(Error::from)?;

        Ok(())
    }
}

// I2C-specific async constructors and methods
#[cfg(feature = "async")]
impl<I2C> Lis2mdlAsync<DeviceInterfaceAsync<I2C>>
where
    I2C: AsyncI2c,
    I2C::Error: Debug,
{
    /// Create a new async I2C driver with the default configuration.
    pub async fn new_i2c(i2c: I2C) -> Result<Self, Error<I2C::Error>> {
        Self::new_i2c_with_config(i2c, Lis2mdlConfig::default()).await
    }

    /// Create a new async I2C driver with an explicit configuration.
    pub async fn new_i2c_with_config(i2c: I2C, config: Lis2mdlConfig) -> Result<Self, Error<I2C::Error>> {
        let interface = DeviceInterfaceAsync {
            i2c,
            address: I2C_ADDRESS,
        };
        let mut device = Lis2mdlDevice::new(interface);
        verify_device_id_async(&mut device).await?;
        let mut this = Self { device, config };
        this.apply_config(config).await?;
        Ok(this)
    }

    /// Access the generated register API directly.
    pub fn device(&mut self) -> &mut Lis2mdlDevice<DeviceInterfaceAsync<I2C>> {
        &mut self.device
    }

    /// Consume the driver and return the underlying asynchronous I2C bus.
    pub fn destroy(self) -> I2C {
        self.device.interface.i2c
    }
}

// Async SPI-specific constructors and methods
#[cfg(feature = "async")]
impl<SPI> Lis2mdlAsync<SpiInterfaceAsync<SPI>>
where
    SPI: AsyncSpiDevice,
    SPI::Error: Debug,
{
    /// Create a new asynchronous SPI driver with the default configuration.
    pub async fn new_spi(spi: SPI) -> Result<Self, Error<SPI::Error>> {
        Self::new_spi_with_config(spi, Lis2mdlConfig::default()).await
    }

    /// Create a new asynchronous SPI driver with an explicit configuration.
    pub async fn new_spi_with_config(spi: SPI, config: Lis2mdlConfig) -> Result<Self, Error<SPI::Error>> {
        let interface = SpiInterfaceAsync { spi };
        let mut device = Lis2mdlDevice::new(interface);
        verify_device_id_async(&mut device).await?;
        let mut this = Self { device, config };
        this.apply_config(config).await?;
        Ok(this)
    }

    /// Access the generated register API directly.
    pub fn device(&mut self) -> &mut Lis2mdlDevice<SpiInterfaceAsync<SPI>> {
        &mut self.device
    }

    /// Consume the driver and return the underlying async SPI bus.
    pub fn destroy(self) -> SPI {
        self.device.interface.spi
    }
}

fn verify_device_id<IFACE>(device: &mut Lis2mdlDevice<IFACE>) -> Result<(), Error<<IFACE as RegisterInterface>::Error>>
where
    IFACE: RegisterInterface<AddressType = u8>,
    <IFACE as RegisterInterface>::Error: Debug,
{
    let who = device.who_am_i().read().map_err(Error::from)?;
    let who_bytes: [u8; 1] = who.into();
    if who_bytes[0] != DEVICE_ID {
        return Err(Error::InvalidDevice);
    }
    Ok(())
}

#[cfg(feature = "async")]
async fn verify_device_id_async<IFACE>(
    device: &mut Lis2mdlDevice<IFACE>,
) -> Result<(), Error<<IFACE as AsyncRegisterInterface>::Error>>
where
    IFACE: AsyncRegisterInterface<AddressType = u8>,
    <IFACE as AsyncRegisterInterface>::Error: Debug,
{
    let who = device.who_am_i().read_async().await.map_err(Error::from)?;
    let who_bytes: [u8; 1] = who.into();
    if who_bytes[0] != DEVICE_ID {
        return Err(Error::InvalidDevice);
    }
    Ok(())
}

fn decode_raw(bytes: &[u8; 6]) -> MagneticField<i16> {
    let x = i16::from_le_bytes([bytes[0], bytes[1]]);
    let y = i16::from_le_bytes([bytes[2], bytes[3]]);
    let z = i16::from_le_bytes([bytes[4], bytes[5]]);
    MagneticField::new(x, y, z)
}

fn scale_to_mgauss(raw: MagneticField<i16>, mgauss_per_lsb: f32) -> MagneticField<i16> {
    fn round_to_i16(value: f32) -> i16 {
        if value >= 0.0 {
            (value + 0.5) as i16
        } else {
            (value - 0.5) as i16
        }
    }

    MagneticField::new(
        round_to_i16(raw.x as f32 * mgauss_per_lsb),
        round_to_i16(raw.y as f32 * mgauss_per_lsb),
        round_to_i16(raw.z as f32 * mgauss_per_lsb),
    )
}

fn scale_to_gauss(raw: MagneticField<i16>, gauss_per_lsb: f32) -> MagneticField<f32> {
    MagneticField::new(
        raw.x as f32 * gauss_per_lsb,
        raw.y as f32 * gauss_per_lsb,
        raw.z as f32 * gauss_per_lsb,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decode_raw_creates_correct_vector() {
        let bytes: [u8; 6] = [0x40, 0x00, 0x80, 0x00, 0xC0, 0x00];
        let decoded = decode_raw(&bytes);

        assert_eq!(decoded.x, 0x0040);
        assert_eq!(decoded.y, 0x0080);
        assert_eq!(decoded.z, 0x00C0);
    }

    #[test]
    fn scale_to_mgauss_rounds_toward_nearest() {
        let raw = MagneticField::new(1, -1, 2);
        let scaled = scale_to_mgauss(raw, 1.5);

        assert_eq!(scaled.x, 2);
        assert_eq!(scaled.y, -2);
        assert_eq!(scaled.z, 3);
    }

    #[test]
    fn scale_to_gauss_scales_components() {
        let raw = MagneticField::new(2, -4, 0);
        let scaled = scale_to_gauss(raw, 0.5);

        assert!((scaled.x - 1.0).abs() <= core::f32::EPSILON);
        assert!((scaled.y + 2.0).abs() <= core::f32::EPSILON);
        assert!(scaled.z.abs() <= core::f32::EPSILON);
    }

    #[test]
    fn status_from_raw_parses_all_fields() {
        // Create a status with specific bit pattern for testing
        let raw_bytes = [0b10101010u8]; // Alternating bits for testing
        let raw: field_sets::StatusReg = raw_bytes.into();

        let status = Status::from_raw(raw);
        // Verify the structure works correctly
        assert_eq!(status.x_ready, raw.xda());
        assert_eq!(status.y_ready, raw.yda());
        assert_eq!(status.z_ready, raw.zda());
        assert_eq!(status.xyz_ready, raw.zyxda());
        assert_eq!(status.x_overrun, raw.xor());
        assert_eq!(status.y_overrun, raw.yor());
        assert_eq!(status.z_overrun, raw.zor());
        assert_eq!(status.xyz_overrun, raw.zyxor());
    }

    #[test]
    fn status_is_ready_checks_all_axes() {
        // Bit 3 is ZYXDA
        let raw_bytes = [0b00001000u8];
        let raw: field_sets::StatusReg = raw_bytes.into();
        let status = Status::from_raw(raw);
        assert!(status.is_ready());
    }

    #[test]
    fn status_has_overrun_checks_any_axis() {
        // Bit 7 is ZYXOR
        let raw_bytes = [0b10000000u8];
        let raw: field_sets::StatusReg = raw_bytes.into();
        let status = Status::from_raw(raw);
        assert!(status.has_overrun());
    }

    #[test]
    fn operating_mode_lp_returns_correct_value() {
        assert!(!OperatingMode::HighResolution.lp());
        assert!(OperatingMode::LowPower.lp());
    }

    #[test]
    fn default_config_has_expected_values() {
        let config = Lis2mdlConfig::default();
        assert_eq!(config.odr, Odr::Hz10);
        assert_eq!(config.mode, OperatingMode::HighResolution);
        assert_eq!(config.measurement_mode, Md::Continuous);
        assert!(config.block_data_update);
        assert!(config.temperature_compensation);
        assert_eq!(config.offset_cancellation, OffsetCancellation::Disabled);
        assert!(!config.low_pass_filter);
    }

    #[test]
    fn sensitivity_constants_are_correct() {
        assert!((SENSITIVITY_MGAUSS_PER_LSB - 1.5).abs() <= core::f32::EPSILON);
        assert!((SENSITIVITY_GAUSS_PER_LSB - 0.0015).abs() <= 0.00001);
    }

    #[test]
    fn magnetic_field_new_creates_instance() {
        let mag = MagneticField::new(100, 200, 300);
        assert_eq!(mag.x, 100);
        assert_eq!(mag.y, 200);
        assert_eq!(mag.z, 300);
    }

    #[test]
    fn temperature_new_creates_instance() {
        let temp = Temperature::new(25.5);
        assert!((temp.celsius - 25.5).abs() <= core::f32::EPSILON);
    }
}
