use gpiod::*;
use pyo3::prelude::*;

#[pyclass]
pub struct SensorChooser {
    pins: Vec<u8>,
    chip: Chip,
    request_state: Vec<(u8, bool)>,
}

#[pymethods]
impl SensorChooser {
    #[new]
    pub fn new(pins: Vec<u8>) -> PyResult<Self> {
        let chip = Chip::new("/dev/gpiochip0").expect("Could not access /dev/gpiochip0");

        let offsets: Vec<u32> = pins.iter().map(|&pin| pin as u32).collect();

        let line_config = Options::output(&offsets).values(&vec![false; pins.len()]);

        chip.request_lines(line_config)
            .expect("Could not initialize sensor to all off state");

        let request_state = pins.iter().map(|&pin| (pin, false)).collect();

        Ok(SensorChooser {
            pins,
            chip,
            request_state,
        })
    }

    pub fn set_pin_states(&mut self, states: Vec<(u8, bool)>) -> PyResult<()> {
        // Validate all pins first
        for (pin, _) in &states {
            if !self.pins.contains(pin) {
                return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(format!(
                    "Pin {} is not in the list of pins set up on initialization",
                    pin
                )));
            }
        }

        let mut new_states: Vec<(u8, bool)> = self.pins.iter().map(|&pin| (pin, false)).collect();

        for (pin, state) in states {
            if let Some(existing_state) = new_states.iter_mut().find(|(p, _)| *p == pin) {
                existing_state.1 = state;
            }
        }

        // Update request state
        self.request_state = new_states;

        // Convert to format needed for GPIO
        let offsets: Vec<u32> = self.pins.iter().map(|&pin| pin as u32).collect();
        let values: Vec<bool> = self.request_state.iter().map(|(_, state)| *state).collect();

        // Update GPIO states
        let line_config = Options::output(&offsets).values(&values);

        self.chip.request_lines(line_config).map_err(|e| {
            PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(format!(
                "Failed to set GPIO states: {}",
                e
            ))
        })?;

        Ok(())
    }
}
