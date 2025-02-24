use linux_embedded_hal::I2cdev;
use pyo3::prelude::*;
use vl53l1x_uld::{DistanceMode, IOVoltage, VL53L1X};

/// Valid timing budget values in milliseconds
const VALID_TIMING_BUDGETS: &[u16] = &[15, 20, 33, 50, 100, 200, 500];

#[pyclass]
#[doc = "VL53L1X Time-of-Flight distance sensor wrapper.\n\n\
         Valid timing budgets (ms): 15, 20, 33, 50, 100, 200, 500\n\
         Note: 15ms is only available in short distance mode.\n\
         Inter-measurement period must be >= timing budget."]
struct TofSensor {
    sensor: VL53L1X<I2cdev>,
}

#[pymethods]
impl TofSensor {
    #[new]
    #[pyo3(signature = (bus, address=None))]
    fn new(bus: u8, address: Option<u8>) -> PyResult<Self> {
        // Validate timing budget

        let i2c = I2cdev::new(format!("/dev/i2c-{}", bus)).map_err(|e| {
            PyErr::new::<pyo3::exceptions::PyIOError, _>(format!("Failed to open I2C bus: {}", e))
        })?;

        let vl53l1x = TofSensor {
            sensor: VL53L1X::new(i2c, address.unwrap_or(0x29)),
        };

        Ok(vl53l1x)
    }

    fn init(&mut self, voltage_2v8: bool) -> PyResult<()> {
        self.sensor
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

    fn start_ranging(&mut self) -> PyResult<()> {
        self.sensor.start_ranging().map_err(|e| {
            PyErr::new::<pyo3::exceptions::PyIOError, _>(format!(
                "Failed to start ranging: {:?}",
                e
            ))
        })
    }

    fn stop_ranging(&mut self) -> PyResult<()> {
        self.sensor.stop_ranging().map_err(|e| {
            PyErr::new::<pyo3::exceptions::PyIOError, _>(format!("Failed to stop ranging: {:?}", e))
        })
    }

    fn is_data_ready(&mut self) -> PyResult<bool> {
        self.sensor.is_data_ready().map_err(|e| {
            PyErr::new::<pyo3::exceptions::PyIOError, _>(format!(
                "Failed to check data ready: {:?}",
                e
            ))
        })
    }

    fn get_distance(&mut self) -> PyResult<u16> {
        self.sensor.get_distance().map_err(|e| {
            PyErr::new::<pyo3::exceptions::PyIOError, _>(format!("Failed to get distance: {:?}", e))
        })
    }

    fn get_range_status(&mut self) -> PyResult<u8> {
        Ok(self.sensor.get_range_status().map_err(|e| {
            PyErr::new::<pyo3::exceptions::PyIOError, _>(format!(
                "Failed to get range status: {:?}",
                e
            ))
        })? as u8)
    }

    fn clear_interrupt(&mut self) -> PyResult<()> {
        self.sensor.clear_interrupt().map_err(|e| {
            PyErr::new::<pyo3::exceptions::PyIOError, _>(format!(
                "Failed to clear interrupt: {:?}",
                e
            ))
        })
    }

    /// Set the timing budget in milliseconds.
    /// Valid values: 15 (short mode only), 20, 33, 50, 100, 200, 500
    fn set_timing_budget_ms(&mut self, budget_ms: u16) -> PyResult<()> {
        if !VALID_TIMING_BUDGETS.contains(&budget_ms) {
            return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(format!(
                "Invalid timing budget. Must be one of: {:?}",
                VALID_TIMING_BUDGETS
            )));
        }

        self.sensor.set_timing_budget_ms(budget_ms).map_err(|e| {
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
    fn set_inter_measurement_period_ms(&mut self, period_ms: u16) -> PyResult<()> {
        let current_budget = self.get_timing_budget_ms()?;

        if period_ms < current_budget + 4 {
            return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(
                format!("Inter-measurement period ({} ms) must be greater than or equal to timing budget ({} ms)",
                    period_ms, current_budget)
            ));
        }

        self.sensor
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

    fn get_timing_budget_ms(&mut self) -> PyResult<u16> {
        self.sensor.get_timing_budget_ms().map_err(|e| {
            PyErr::new::<pyo3::exceptions::PyIOError, _>(format!(
                "Failed to get timing budget: {:?}",
                e
            ))
        })
    }

    /// Get the current inter-measurement period in milliseconds
    fn get_inter_measurement_period_ms(&mut self) -> PyResult<u16> {
        self.sensor.get_inter_measurement_period_ms().map_err(|e| {
            PyErr::new::<pyo3::exceptions::PyIOError, _>(format!(
                "Failed to get inter-measurement period: {:?}",
                e
            ))
        })
    }

    fn set_long_distance_mode(&mut self, long_range: bool) -> PyResult<()> {
        self.sensor
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
    fn read_bytes(&mut self, reg: u16, len: usize) -> PyResult<Vec<u8>> {
        let a = reg.to_be_bytes();
        let mut buf = vec![0; len];
        self.sensor.read_bytes(a, &mut buf).map_err(|e| {
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
