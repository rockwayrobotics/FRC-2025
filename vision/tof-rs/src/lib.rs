use crossbeam_channel::{bounded, Receiver, Sender, TrySendError};
use linux_embedded_hal::I2cdev;
use nix::time::{clock_gettime, ClockId};
use pyo3::create_exception;
use pyo3::exceptions::{PyException, PyIOError, PyOverflowError, PyValueError};
use pyo3::prelude::*;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc, Mutex,
};
use std::thread;
use std::time::{Duration, Instant};
use thread_priority::{ThreadBuilder, ThreadPriority};
use vl53l1x_uld::roi::{ROICenter, ROI};
use vl53l1x_uld::{DistanceMode, IOVoltage, VL53L1X};

// use crate::sensor_chooser::SensorChooser;
mod set_pins;

/// Valid timing budget values in milliseconds
const VALID_TIMING_BUDGETS: &[u16] = &[15, 20, 33, 50, 100, 200, 500];

/// Channel for readings and errors. Expected to need only a little space
/// provided the Python can keep up.  Otherwise needs space for as many
/// readings as can be retrieved while the Python is busy.
const CHANNEL_SIZE: usize = 10;

// Define a custom exception for thread errors
create_exception!(vl53l1x, ThreadError, PyException);

// Enhanced reading structure
#[derive(Clone)]
struct SensorReading {
    timestamp: f64,
    distance: u16,
    status: u8,
}

// Message types for the channel
enum SensorResult {
    Reading(SensorReading),     // Normal reading
    I2CError(String),           // I2C communication error
    ThreadExit(Option<String>), // Normal or unexpected exit
}

// The main ToF sensor class
#[pyclass]
#[doc = "VL53L1X Time-of-Flight distance sensor wrapper.\n\n\
         Valid timing budgets (ms): 15, 20, 33, 50, 100, 200, 500\n\
         Note: 15ms is only available in short distance mode.\n\
         Inter-measurement period must be >= timing budget."]
struct TofSensor {
    sensor: Arc<Mutex<VL53L1X<I2cdev>>>,

    // Channel for communication between thread and Python
    reading_sender: Option<Sender<SensorResult>>,
    reading_receiver: Option<Receiver<SensorResult>>,

    // Thread management
    thread_handle: Option<thread::JoinHandle<()>>,
    is_running: Arc<AtomicBool>,

    // Flags for error conditions
    overflow_flag: Arc<Mutex<bool>>,
}

#[pymethods]
impl TofSensor {
    #[new]
    #[pyo3(signature = (bus, address=None))]
    fn new(bus: u8, address: Option<u8>) -> PyResult<Self> {
        let i2c = I2cdev::new(format!("/dev/i2c-{}", bus))
            .map_err(|e| PyErr::new::<PyIOError, _>(format!("Failed to open I2C bus: {}", e)))?;

        Ok(TofSensor {
            sensor: Arc::new(Mutex::new(VL53L1X::new(i2c, address.unwrap_or(0x29)))),
            reading_sender: None,
            reading_receiver: None,
            thread_handle: None,
            is_running: Arc::new(AtomicBool::new(false)),
            overflow_flag: Arc::new(Mutex::new(false)),
        })
    }

    // Basic sensor configuration methods
    fn init(&self, voltage_2v8: bool) -> PyResult<()> {
        let mut sensor = self.sensor.lock().unwrap();
        sensor
            .init(if voltage_2v8 {
                IOVoltage::Volt2_8
            } else {
                IOVoltage::Volt1_8
            })
            .map_err(|e| {
                PyErr::new::<PyIOError, _>(format!("Failed to initialize sensor: {:?}", e))
            })
    }

    fn start_ranging(&self) -> PyResult<()> {
        let mut sensor = self.sensor.lock().unwrap();
        sensor
            .start_ranging()
            .map_err(|e| PyErr::new::<PyIOError, _>(format!("Failed to start ranging: {:?}", e)))
    }

    fn stop_ranging(&self) -> PyResult<()> {
        let mut sensor = self.sensor.lock().unwrap();
        sensor
            .stop_ranging()
            .map_err(|e| PyErr::new::<PyIOError, _>(format!("Failed to stop ranging: {:?}", e)))
    }

    fn is_data_ready(&self) -> PyResult<bool> {
        let mut sensor = self.sensor.lock().unwrap();
        sensor
            .is_data_ready()
            .map_err(|e| PyErr::new::<PyIOError, _>(format!("Failed to check data ready: {:?}", e)))
    }

    fn get_distance(&self) -> PyResult<u16> {
        let mut sensor = self.sensor.lock().unwrap();
        sensor
            .get_distance()
            .map_err(|e| PyErr::new::<PyIOError, _>(format!("Failed to get distance: {:?}", e)))
    }

    fn get_range_status(&self) -> PyResult<u8> {
        let mut sensor = self.sensor.lock().unwrap();
        Ok(sensor.get_range_status().map_err(|e| {
            PyErr::new::<PyIOError, _>(format!("Failed to get range status: {:?}", e))
        })? as u8)
    }

    fn clear_interrupt(&self) -> PyResult<()> {
        let mut sensor = self.sensor.lock().unwrap();
        sensor
            .clear_interrupt()
            .map_err(|e| PyErr::new::<PyIOError, _>(format!("Failed to clear interrupt: {:?}", e)))
    }

    fn set_timing_budget_ms(&self, budget_ms: u16) -> PyResult<()> {
        if !VALID_TIMING_BUDGETS.contains(&budget_ms) {
            return Err(PyErr::new::<PyValueError, _>(format!(
                "Invalid timing budget. Must be one of: {:?}",
                VALID_TIMING_BUDGETS
            )));
        }

        let mut sensor = self.sensor.lock().unwrap();
        sensor.set_timing_budget_ms(budget_ms).map_err(|e| {
            PyErr::new::<PyValueError, _>(format!("Failed to set timing budget: {:?}", e))
        })
    }

    fn set_inter_measurement_period_ms(&self, period_ms: u16) -> PyResult<()> {
        let mut sensor = self.sensor.lock().unwrap();
        let current_budget = sensor.get_timing_budget_ms().map_err(|e| {
            PyErr::new::<PyIOError, _>(format!("Failed to get timing budget: {:?}", e))
        })?;

        if period_ms < current_budget {
            return Err(PyErr::new::<PyValueError, _>(
                format!("Inter-measurement period ({} ms) must be greater than or equal to timing budget ({} ms)",
                    period_ms, current_budget)
            ));
        }

        sensor
            .set_inter_measurement_period_ms(period_ms)
            .map_err(|e| {
                PyErr::new::<PyValueError, _>(format!(
                    "Failed to set inter-measurement period: {:?}",
                    e
                ))
            })?;

        let actual_period = sensor.get_inter_measurement_period_ms().map_err(|e| {
            PyErr::new::<PyIOError, _>(format!("Failed to get inter-measurement period: {:?}", e))
        })?;
        if actual_period != period_ms {
            println!("actual_inter_measurement_period: {}", actual_period);
            return Err(PyErr::new::<PyValueError, _>(format!(
                "Setting inter-measurement period ({} ms) failed (actual {} ms)",
                period_ms, actual_period
            )));
        }

        Ok(())
    }

    fn get_timing_budget_ms(&self) -> PyResult<u16> {
        let mut sensor = self.sensor.lock().unwrap();
        sensor.get_timing_budget_ms().map_err(|e| {
            PyErr::new::<PyIOError, _>(format!("Failed to get timing budget: {:?}", e))
        })
    }

    fn get_inter_measurement_period_ms(&self) -> PyResult<u16> {
        let mut sensor = self.sensor.lock().unwrap();
        sensor.get_inter_measurement_period_ms().map_err(|e| {
            PyErr::new::<PyIOError, _>(format!("Failed to get inter-measurement period: {:?}", e))
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
                PyErr::new::<PyValueError, _>(format!("Failed to set distance mode: {:?}", e))
            })
    }

    fn set_roi(&self, width: u16, height: u16) -> PyResult<()> {
        let mut sensor = self.sensor.lock().unwrap();
        sensor
            .set_roi(ROI {
                width: width,
                height: height,
            })
            .map_err(|e| PyErr::new::<PyValueError, _>(format!("Failed to set ROI: {:?}", e)))
    }

    fn set_roi_center(&self, x: u8, y: u8) -> PyResult<()> {
        let mut sensor = self.sensor.lock().unwrap();
        let roi_center = ROICenter::new(x, y);
        sensor.set_roi_center(roi_center).map_err(|e| {
            PyErr::new::<PyValueError, _>(format!("Failed to set ROI center: {:?}", e))
        })
    }

    // Enhanced streaming methods with channel-based communication
    fn start_streaming(&mut self) -> PyResult<()> {
        // Return early if already streaming
        if self.is_running.load(Ordering::Relaxed) {
            return Ok(());
        }

        // Reset the overflow flag
        *self.overflow_flag.lock().unwrap() = false;

        // Create a bounded channel with appropriate capacity
        // Adjust capacity based on your needs and memory constraints
        let (sender, receiver) = bounded(CHANNEL_SIZE);
        self.reading_sender = Some(sender);
        self.reading_receiver = Some(receiver);

        // Create clones of the shared resources for the thread
        let sensor = Arc::clone(&self.sensor);
        let is_running = Arc::clone(&self.is_running);
        let overflow_flag = Arc::clone(&self.overflow_flag);
        let sender = self.reading_sender.as_ref().unwrap().clone();

        // Set the running flag before starting thread
        is_running.store(true, Ordering::SeqCst);

        // Initialize the sensor for ranging
        {
            let mut sensor = sensor.lock().unwrap();
            if let Err(e) = sensor.start_ranging() {
                return Err(PyErr::new::<PyIOError, _>(format!(
                    "Failed to start ranging: {:?}",
                    e
                )));
            }
        }

        let thread_is_running = is_running.clone();
        // Spawn thread with high priority
        let thread = ThreadBuilder::default()
            .name("tof_sensor_thread".to_string())
            .priority(ThreadPriority::Max)
            .spawn(move |_| {
                // Worker thread implementation
                let _ = reading_loop(sensor, thread_is_running.clone(), overflow_flag, sender);

                // Always mark as not running when thread exits
                thread_is_running.store(false, Ordering::SeqCst);
            })
            .map_err(|e| {
                is_running.store(false, Ordering::SeqCst);
                PyErr::new::<PyException, _>(format!("Failed to spawn sensor thread: {}", e))
            })?;

        self.thread_handle = Some(thread);
        Ok(())
    }

    fn get_reading(&self, py: Python<'_>) -> PyResult<Option<(f64, u16, u8)>> {
        // Get receiver
        let receiver = match &self.reading_receiver {
            Some(r) => r,
            None => {
                return Err(PyErr::new::<PyIOError, _>(
                    "Sensor not streaming".to_string(),
                ))
            }
        };

        // If channel is empty and thread is not running, check for overflow first
        if !self.is_running.load(Ordering::Relaxed) && receiver.is_empty() {
            if *self.overflow_flag.lock().unwrap() {
                return Err(PyErr::new::<PyOverflowError, _>(
                    "Sensor reading channel overflow".to_string(),
                ));
            }
            // Normal thread exit with empty channel
            return Ok(None);
        }

        // Block and allow Python to release GIL while waiting
        let result = py.allow_threads(|| receiver.recv());

        match result {
            Ok(SensorResult::Reading(reading)) => {
                Ok(Some((reading.timestamp, reading.distance, reading.status)))
            }
            Ok(SensorResult::I2CError(err)) => Err(PyErr::new::<PyIOError, _>(err)),
            Ok(SensorResult::ThreadExit(None)) => {
                // Normal exit
                Ok(None)
            }
            Ok(SensorResult::ThreadExit(Some(err))) => {
                // Error exit
                Err(ThreadError::new_err(err))
            }
            Err(_) => {
                // Channel closed unexpectedly
                if *self.overflow_flag.lock().unwrap() {
                    Err(PyErr::new::<PyOverflowError, _>(
                        "Sensor reading channel overflow".to_string(),
                    ))
                } else {
                    // Normal shutdown
                    Ok(None)
                }
            }
        }
    }

    fn is_running(&self) -> bool {
        self.is_running.load(Ordering::Relaxed)
    }

    fn stop_streaming(&mut self) -> PyResult<()> {
        // Signal thread to stop
        self.is_running.store(false, Ordering::SeqCst);

        // Try to send a normal exit message
        if let Some(sender) = &self.reading_sender {
            let _ = sender.try_send(SensorResult::ThreadExit(None));
        }

        // Join thread if possible
        if let Some(handle) = self.thread_handle.take() {
            handle.join().ok();
        }

        // Stop the sensor
        let mut sensor = self.sensor.lock().unwrap();
        let _ = sensor.stop_ranging();

        // Clean up channel
        self.reading_sender = None;
        self.reading_receiver = None;

        Ok(())
    }
}

fn get_monotime() -> f64 {
    match clock_gettime(ClockId::CLOCK_MONOTONIC) {
        Ok(ts) => ts.tv_sec() as f64 + (ts.tv_nsec() as f64 / 1_000_000_000.0),

        Err(_) => 0.0, // never expecting this
    }
}

// Worker function for the sensor reading thread
fn reading_loop(
    sensor: Arc<Mutex<VL53L1X<I2cdev>>>,
    is_running: Arc<AtomicBool>,
    overflow_flag: Arc<Mutex<bool>>,
    sender: Sender<SensorResult>,
) -> Result<(), String> {
    // Retrieve timing parameters so we know how long to wait.
    let timing_budget = {
        let mut sensor = match sensor.lock() {
            Ok(s) => s,
            Err(_) => return Err("Failed to lock sensor mutex".to_string()),
        };
        match sensor.get_timing_budget_ms() {
            Ok(tb) => tb,
            Err(e) => return Err(format!("Failed to get timing budget: {:?}", e)),
        }
    } as u64;

    // Delay this much after each check. The idea is we start with no delay
    // but then delay a short time (rapid poll) until we get a reading,
    // then wait a bit less than the timing budget until we get another.
    // That's more efficient than busy-waiting all the time.
    let mut delay = 0;
    let mut ts = Instant::now();

    // Main reading loop
    while is_running.load(Ordering::Relaxed) {
        // Check if data is ready
        let mut sensor_guard = match sensor.lock() {
            Ok(s) => s,
            Err(_) => {
                // Mutex poisoned, report error and exit
                let err = "Sensor mutex poisoned".to_string();
                let _ = sender.try_send(SensorResult::ThreadExit(Some(err.clone())));
                return Err(err);
            }
        };

        match sensor_guard.is_data_ready() {
            Ok(true) => {
                // Initially pause a bit less than the timing budget, at which
                // point we'll begin more frequent checks.
                delay = timing_budget.saturating_sub(5) + 1;
                // delay = 1;

                // Data is ready, try to read distance
                match sensor_guard.get_distance() {
                    Ok(distance) => {
                        // Get range status
                        let status = match sensor_guard.get_range_status() {
                            Ok(val) => val as u8,
                            _ => 0,
                        };
                        // Notes
                        // 0 = good reading
                        // 1 = sigma failure (noisy signal), increase timing budget?
                        // 2 = signal failure, signal too weak, target too far
                        //     or not reflective enough or too small (or missing)
                        // 4 = out of bounds (warning), at upper limit of dist
                        // 7 = wraparound, also past upper end of range
                        // We get solid 0s with an object within valid range.
                        // We can probably ignore 1 (but logging them may be
                        // useful), and treat 2, 4, 7 as errors but possibly
                        // just convert reading to a large value so our distance
                        // filtering just ignores it.

                        // Create reading
                        let reading = SensorReading {
                            timestamp: get_monotime(),
                            distance,
                            status,
                        };

                        // Clear the interrupt
                        let _ = sensor_guard.clear_interrupt();

                        let delta = Instant::now() - ts;
                        ts += delta;
                        // println!(
                        //     "delta {:>2}, dist {:>4}, status {:>2}",
                        //     delta.as_millis(),
                        //     distance,
                        //     status
                        // );

                        // Drop the lock before potentially blocking on send
                        // drop(sensor_guard);

                        // Try to send reading, handle channel overflow
                        match sender.try_send(SensorResult::Reading(reading)) {
                            Ok(_) => {
                                // Successfully sent reading
                            }
                            Err(TrySendError::Full(_)) => {
                                // Channel full - critical error condition
                                *overflow_flag.lock().unwrap() = true;
                                return Err("Reading channel overflow".to_string());
                            }
                            Err(TrySendError::Disconnected(_)) => {
                                // Receiver dropped - exit normally
                                return Ok(());
                            }
                        }
                    }
                    Err(e) => {
                        // I2C error reading distance
                        let err = format!("Failed to get distance: {:?}", e);
                        // drop(sensor_guard);

                        // Try to send error
                        let _ = sender.try_send(SensorResult::I2CError(err.clone()));
                        return Err(err);
                    }
                }
            }
            Ok(false) => {
                // No data ready, just wait
                // drop(sensor_guard);
            }
            Err(e) => {
                // I2C error checking data ready
                let err = format!("Error checking data ready: {:?}", e);
                // drop(sensor_guard);

                // Try to send error
                let _ = sender.try_send(SensorResult::I2CError(err.clone()));
                return Err(err);
            }
        }

        drop(sensor_guard);

        // Pause to avoid busy wait.  The initial pause after a reading
        // is just shorter than the timing budget, and as soon as we've
        // done that long a delay we'll adjust so subsequent delays are
        // short until we get a reading.
        thread::sleep(Duration::from_millis(delay));
        delay = 1;
    }

    // Normal exit
    let _ = sender.try_send(SensorResult::ThreadExit(None));
    Ok(())
}

// Register the module
#[pymodule]
fn vl53l1x(py: Python, m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<TofSensor>()?;
    m.add_class::<set_pins::SetPins>()?;
    m.add("ThreadError", py.get_type::<ThreadError>())?;
    Ok(())
}
