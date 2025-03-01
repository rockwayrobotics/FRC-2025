use linux_embedded_hal::I2cdev;
use pyo3::prelude::*;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};
use thread_priority::{ThreadBuilder, ThreadPriority};
use vl53l1x_uld::{DistanceMode, IOVoltage, VL53L1X};

// use crate::sensor_chooser::SensorChooser;
mod set_pins;
/// Valid timing budget values in milliseconds
const VALID_TIMING_BUDGETS: &[u16] = &[15, 20, 33, 50, 100, 200, 500];

#[derive(Clone)]
struct SensorReading {
    timestamp: f64,
    distance: u16,
}

#[pyclass]
#[doc = "VL53L1X Time-of-Flight distance sensor wrapper.\n\n\
         Valid timing budgets (ms): 15, 20, 33, 50, 100, 200, 500\n\
         Note: 15ms is only available in short distance mode.\n\
         Inter-measurement period must be >= timing budget."]
struct TofSensor {
    sensor: Arc<Mutex<VL53L1X<I2cdev>>>,
    latest_reading: Arc<Mutex<Option<SensorReading>>>,
    is_streaming: Arc<Mutex<bool>>,
}

#[pymethods]
impl TofSensor {
    #[new]
    #[pyo3(signature = (bus, address=None))]
    fn new(bus: u8, address: Option<u8>) -> PyResult<Self> {
        let i2c = I2cdev::new(format!("/dev/i2c-{}", bus)).map_err(|e| {
            PyErr::new::<pyo3::exceptions::PyIOError, _>(format!("Failed to open I2C bus: {}", e))
        })?;

        Ok(TofSensor {
            sensor: Arc::new(Mutex::new(VL53L1X::new(i2c, address.unwrap_or(0x29)))),
            latest_reading: Arc::new(Mutex::new(None)),
            is_streaming: Arc::new(Mutex::new(false)),
        })
    }

    fn init(&self, voltage_2v8: bool) -> PyResult<()> {
        let mut sensor = self.sensor.lock().unwrap();
        sensor
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
        let mut sensor = self.sensor.lock().unwrap();
        sensor.start_ranging().map_err(|e| {
            PyErr::new::<pyo3::exceptions::PyIOError, _>(format!(
                "Failed to start ranging: {:?}",
                e
            ))
        })
    }

    fn stop_ranging(&self) -> PyResult<()> {
        let mut sensor = self.sensor.lock().unwrap();
        sensor.stop_ranging().map_err(|e| {
            PyErr::new::<pyo3::exceptions::PyIOError, _>(format!("Failed to stop ranging: {:?}", e))
        })
    }

    fn is_data_ready(&self) -> PyResult<bool> {
        let mut sensor = self.sensor.lock().unwrap();
        sensor.is_data_ready().map_err(|e| {
            PyErr::new::<pyo3::exceptions::PyIOError, _>(format!(
                "Failed to check data ready: {:?}",
                e
            ))
        })
    }

    fn get_distance(&self) -> PyResult<u16> {
        let mut sensor = self.sensor.lock().unwrap();
        sensor.get_distance().map_err(|e| {
            PyErr::new::<pyo3::exceptions::PyIOError, _>(format!("Failed to get distance: {:?}", e))
        })
    }

    fn get_range_status(&self) -> PyResult<u8> {
        let mut sensor = self.sensor.lock().unwrap();
        Ok(sensor.get_range_status().map_err(|e| {
            PyErr::new::<pyo3::exceptions::PyIOError, _>(format!(
                "Failed to get range status: {:?}",
                e
            ))
        })? as u8)
    }

    fn clear_interrupt(&self) -> PyResult<()> {
        let mut sensor = self.sensor.lock().unwrap();
        sensor.clear_interrupt().map_err(|e| {
            PyErr::new::<pyo3::exceptions::PyIOError, _>(format!(
                "Failed to clear interrupt: {:?}",
                e
            ))
        })
    }

    fn set_timing_budget_ms(&self, budget_ms: u16) -> PyResult<()> {
        if !VALID_TIMING_BUDGETS.contains(&budget_ms) {
            return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(format!(
                "Invalid timing budget. Must be one of: {:?}",
                VALID_TIMING_BUDGETS
            )));
        }

        let mut sensor = self.sensor.lock().unwrap();
        sensor.set_timing_budget_ms(budget_ms).map_err(|e| {
            PyErr::new::<pyo3::exceptions::PyValueError, _>(format!(
                "Failed to set timing budget: {:?}",
                e
            ))
        })?;

        // Verify the setting was applied correctly
        let actual_budget = sensor.get_timing_budget_ms().map_err(|e| {
            PyErr::new::<pyo3::exceptions::PyIOError, _>(format!(
                "Failed to get timing budget: {:?}",
                e
            ))
        })?;
        println!("actual_timing_budget: {}", actual_budget);

        Ok(())
    }

    fn set_inter_measurement_period_ms(&self, period_ms: u16) -> PyResult<()> {
        let mut sensor = self.sensor.lock().unwrap();
        let current_budget = sensor.get_timing_budget_ms().map_err(|e| {
            PyErr::new::<pyo3::exceptions::PyIOError, _>(format!(
                "Failed to get timing budget: {:?}",
                e
            ))
        })?;

        if period_ms < current_budget + 4 {
            return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(
                format!("Inter-measurement period ({} ms) must be greater than or equal to timing budget ({} ms)",
                    period_ms, current_budget)
            ));
        }

        sensor
            .set_inter_measurement_period_ms(period_ms)
            .map_err(|e| {
                PyErr::new::<pyo3::exceptions::PyValueError, _>(format!(
                    "Failed to set inter-measurement period: {:?}",
                    e
                ))
            })?;

        let actual_period = sensor.get_inter_measurement_period_ms().map_err(|e| {
            PyErr::new::<pyo3::exceptions::PyIOError, _>(format!(
                "Failed to get inter-measurement period: {:?}",
                e
            ))
        })?;
        println!("actual_inter_measurement_period: {}", actual_period);

        Ok(())
    }

    fn get_timing_budget_ms(&self) -> PyResult<u16> {
        let mut sensor = self.sensor.lock().unwrap();
        sensor.get_timing_budget_ms().map_err(|e| {
            PyErr::new::<pyo3::exceptions::PyIOError, _>(format!(
                "Failed to get timing budget: {:?}",
                e
            ))
        })
    }

    fn get_inter_measurement_period_ms(&self) -> PyResult<u16> {
        let mut sensor = self.sensor.lock().unwrap();
        sensor.get_inter_measurement_period_ms().map_err(|e| {
            PyErr::new::<pyo3::exceptions::PyIOError, _>(format!(
                "Failed to get inter-measurement period: {:?}",
                e
            ))
        })
    }

    fn set_long_distance_mode(&self, long_range: bool) -> PyResult<()> {
        let mut sensor = self.sensor.lock().unwrap();
        sensor
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
        let mut sensor = self.sensor.lock().unwrap();
        let a = reg.to_be_bytes();
        let mut buf = vec![0; len];
        sensor.read_bytes(a, &mut buf).map_err(|e| {
            PyErr::new::<pyo3::exceptions::PyValueError, _>(format!(
                "Failed to read bytes: {:?}",
                e
            ))
        })?;
        Ok(buf)
    }

    // Streaming methods (from previous implementation)
    fn start_streaming(&mut self) -> PyResult<()> {
        let sensor = Arc::clone(&self.sensor);
        let latest_reading = Arc::clone(&self.latest_reading);
        let is_streaming = Arc::clone(&self.is_streaming);

        {
            let mut streaming = is_streaming.lock().unwrap();
            if *streaming {
                return Ok(());
            }
            *streaming = true;
        }

        // Start the sensor
        {
            let mut sensor = sensor.lock().unwrap();
            sensor.start_ranging().map_err(|e| {
                PyErr::new::<pyo3::exceptions::PyIOError, _>(format!(
                    "Failed to start ranging: {:?}",
                    e
                ))
            })?;
        }

        let start_time = Instant::now(); // Reference point for monotonic time

        // Create thread with highest possible priority
        ThreadBuilder::default()
            .name("tof_sensor_thread".to_string())
            .priority(ThreadPriority::Max) // Set the thread to maximum priority
            .spawn(move |_| {
                // Log the current thread priority - useful for debugging
                println!(
                    "ToF sensor thread started with priority: {:?}",
                    thread_priority::get_current_thread_priority().unwrap_or(ThreadPriority::Min)
                );

                let timing_budget = {
                    let mut sensor = sensor.lock().unwrap();
                    sensor.get_timing_budget_ms().unwrap_or(20)
                };
                let mut last = 0;
                while *is_streaming.lock().unwrap() {
                    if last >= timing_budget {
                        let mut sensor = sensor.lock().unwrap();
                        if sensor.is_data_ready().unwrap_or(false) {
                            if let Ok(distance) = sensor.get_distance() {
                                let reading = SensorReading {
                                    timestamp: Instant::now().duration_since(start_time).as_millis()
                                        as f64, // Monotonic time since start
                                    distance,
                                };
                                *latest_reading.lock().unwrap() = Some(reading);
                                sensor.clear_interrupt().ok();
                                last = 0;
                            }
                        }
                    }
                    last += 1;
                    if last < timing_budget - 1 {
                        thread::sleep(Duration::from_millis(1));
                    } else {
                        thread::sleep(Duration::from_micros(100));
                    }
                }

                // Stop ranging when streaming ends
                if let Ok(mut sensor) = sensor.lock() {
                    let _ = sensor.stop_ranging();
                }
            })
            .map_err(|e| {
                PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(format!(
                    "Failed to spawn sensor thread: {}",
                    e
                ))
            })?;

        Ok(())
    }

    fn get_latest_reading(&self) -> PyResult<Option<(f64, u16)>> {
        let reading = self.latest_reading.lock().unwrap();
        Ok(reading.as_ref().map(|r| (r.timestamp, r.distance)))
    }

    fn stop_streaming(&mut self) -> PyResult<()> {
        let mut is_streaming = self.is_streaming.lock().unwrap();
        *is_streaming = false;
        Ok(())
    }
}

#[pymodule]
fn vl53l1x(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<TofSensor>()?;
    m.add_class::<set_pins::SetPins>()?;
    Ok(())
}
