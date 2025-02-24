mod interrupt;
mod sensor;
mod types;

pub use interrupt::InterruptHandler;
pub use sensor::SensorInterface;
pub use types::{Error, TimestampedDistance};

use linux_embedded_hal::I2cdev;
use pyo3::prelude::*;
use std::sync::{Arc, Mutex};
use vl53l1x_uld::{DistanceMode, IOVoltage, VL53L1X};

const VALID_TIMING_BUDGETS: &[u16] = &[15, 20, 33, 50, 100, 200, 500];

// Implement SensorInterface for our VL53L1X sensor
impl SensorInterface for VL53L1X<I2cdev> {
    fn get_distance(&self) -> Result<u16, Box<dyn std::error::Error>> {
        self.get_distance().map_err(|e| {
            Box::new(std::io::Error::new(
                std::io::ErrorKind::Other,
                format!("{:?}", e),
            )) as Box<dyn std::error::Error>
        })
    }

    fn clear_interrupt(&self) -> Result<(), Box<dyn std::error::Error>> {
        self.clear_interrupt().map_err(|e| {
            Box::new(std::io::Error::new(
                std::io::ErrorKind::Other,
                format!("{:?}", e),
            )) as Box<dyn std::error::Error>
        })
    }
}

#[pyclass]
struct TofSensor {
    sensor: Arc<Mutex<VL53L1X<I2cdev>>>,
    interrupt_handler: Option<InterruptHandler>,
}

#[pymethods]
impl TofSensor {
    #[new]
    fn new(bus: u8, address: Option<u8>) -> PyResult<Self> {
        let i2c = I2cdev::new(format!("/dev/i2c-{}", bus)).map_err(|e| {
            PyErr::new::<pyo3::exceptions::PyIOError, _>(format!("Failed to open I2C bus: {}", e))
        })?;

        let sensor = VL53L1X::new(i2c, address.unwrap_or(0x29));

        Ok(TofSensor {
            sensor: Arc::new(Mutex::new(sensor)),
            interrupt_handler: None,
        })
    }

    fn start_interrupt_handler(
        &mut self,
        chip: &str,
        line: u32,
        queue_size: usize,
    ) -> PyResult<i32> {
        let handler =
            InterruptHandler::new(self.sensor.clone(), chip, line, queue_size).map_err(|e| {
                PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(format!(
                    "Failed to create interrupt handler: {}",
                    e
                ))
            })?;

        let fd = handler.get_fd();
        self.interrupt_handler = Some(handler);
        Ok(fd)
    }

    fn get_interrupt_reading(&mut self) -> PyResult<Option<(f64, u16)>> {
        if let Some(handler) = &self.interrupt_handler {
            Ok(handler.get_reading().map(|reading| {
                let timestamp = reading
                    .timestamp
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap()
                    .as_secs_f64();
                (timestamp, reading.distance)
            }))
        } else {
            Err(PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(
                "Interrupt handler not started",
            ))
        }
    }

    fn init(&self, voltage_2v8: bool) -> PyResult<()> {
        self.sensor
            .lock()
            .unwrap()
            .init(if voltage_2v8 {
                IOVoltage::Volt2_8
            } else {
                IOVoltage::Volt1_8
            })
            .map_err(|e| {
                PyErr::new::<pyo3::exceptions::PyIOError, _>(format!(
                    "Failed to initialize sensor: {:?}",
                    e
                ))
            })
    }

    fn start_ranging(&self) -> PyResult<()> {
        self.sensor.lock().unwrap().start_ranging().map_err(|e| {
            PyErr::new::<pyo3::exceptions::PyIOError, _>(format!(
                "Failed to start ranging: {:?}",
                e
            ))
        })
    }

    fn stop_ranging(&self) -> PyResult<()> {
        self.sensor.lock().unwrap().stop_ranging().map_err(|e| {
            PyErr::new::<pyo3::exceptions::PyIOError, _>(format!("Failed to stop ranging: {:?}", e))
        })
    }

    fn is_data_ready(&self) -> PyResult<bool> {
        self.sensor.lock().unwrap().is_data_ready().map_err(|e| {
            PyErr::new::<pyo3::exceptions::PyIOError, _>(format!(
                "Failed to check data ready: {:?}",
                e
            ))
        })
    }

    fn get_distance(&self) -> PyResult<u16> {
        self.sensor.lock().unwrap().get_distance().map_err(|e| {
            PyErr::new::<pyo3::exceptions::PyIOError, _>(format!("Failed to get distance: {:?}", e))
        })
    }

    fn get_range_status(&self) -> PyResult<u8> {
        Ok(self
            .sensor
            .lock()
            .unwrap()
            .get_range_status()
            .map_err(|e| {
                PyErr::new::<pyo3::exceptions::PyIOError, _>(format!(
                    "Failed to get range status: {:?}",
                    e
                ))
            })? as u8)
    }

    fn clear_interrupt(&self) -> PyResult<()> {
        self.sensor.lock().unwrap().clear_interrupt().map_err(|e| {
            PyErr::new::<pyo3::exceptions::PyIOError, _>(format!(
                "Failed to clear interrupt: {:?}",
                e
            ))
        })
    }

    /// Set the timing budget in milliseconds.
    /// Valid values: 15 (short mode only), 20, 33, 50, 100, 200, 500
    fn set_timing_budget_ms(&self, budget_ms: u16) -> PyResult<()> {
        if !VALID_TIMING_BUDGETS.contains(&budget_ms) {
            return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(format!(
                "Invalid timing budget. Must be one of: {:?}",
                VALID_TIMING_BUDGETS
            )));
        }

        self.sensor
            .lock()
            .unwrap()
            .set_timing_budget_ms(budget_ms)
            .map_err(|e| {
                PyErr::new::<pyo3::exceptions::PyValueError, _>(format!(
                    "Failed to set timing budget: {:?}",
                    e
                ))
            })?;

        // Verify the setting was applied correctly
        let actual_budget = self.get_timing_budget_ms()?;
        println!("actual_timing_budget: {}", actual_budget);

        Ok(())
    }

    /// Set the inter-measurement period in milliseconds.
    /// Must be greater than or equal to the timing budget.
    fn set_inter_measurement_period_ms(&self, period_ms: u16) -> PyResult<()> {
        let current_budget = self.get_timing_budget_ms()?;

        if period_ms < current_budget + 4 {
            return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(
                format!("Inter-measurement period ({} ms) must be greater than or equal to timing budget ({} ms)",
                    period_ms, current_budget)
            ));
        }

        self.sensor
            .lock()
            .unwrap()
            .set_inter_measurement_period_ms(period_ms)
            .map_err(|e| {
                PyErr::new::<pyo3::exceptions::PyValueError, _>(format!(
                    "Failed to set inter-measurement period: {:?}",
                    e
                ))
            })?;

        let actual_period = self.get_inter_measurement_period_ms()?;
        println!("actual_inter_measurement_period: {}", actual_period);

        Ok(())
    }

    fn get_timing_budget_ms(&self) -> PyResult<u16> {
        self.sensor
            .lock()
            .unwrap()
            .get_timing_budget_ms()
            .map_err(|e| {
                PyErr::new::<pyo3::exceptions::PyIOError, _>(format!(
                    "Failed to get timing budget: {:?}",
                    e
                ))
            })
    }

    fn get_inter_measurement_period_ms(&self) -> PyResult<u16> {
        self.sensor
            .lock()
            .unwrap()
            .get_inter_measurement_period_ms()
            .map_err(|e| {
                PyErr::new::<pyo3::exceptions::PyIOError, _>(format!(
                    "Failed to get inter-measurement period: {:?}",
                    e
                ))
            })
    }

    fn set_long_distance_mode(&self, long_range: bool) -> PyResult<()> {
        self.sensor
            .lock()
            .unwrap()
            .set_distance_mode(if long_range {
                DistanceMode::Long
            } else {
                DistanceMode::Short
            })
            .map_err(|e| {
                PyErr::new::<pyo3::exceptions::PyValueError, _>(format!(
                    "Failed to set distance mode: {:?}",
                    e
                ))
            })
    }

    fn read_bytes(&self, reg: u16, len: usize) -> PyResult<Vec<u8>> {
        let a = reg.to_be_bytes();
        let mut buf = vec![0; len];
        self.sensor
            .lock()
            .unwrap()
            .read_bytes(a, &mut buf)
            .map_err(|e| {
                PyErr::new::<pyo3::exceptions::PyValueError, _>(format!(
                    "Failed to read bytes: {:?}",
                    e
                ))
            })?;
        Ok(buf)
    }
}

#[pymodule]
fn vl53l1x(_py: Python<'_>, m: &PyModule) -> PyResult<()> {
    m.add_class::<TofSensor>()?;
    Ok(())
}
