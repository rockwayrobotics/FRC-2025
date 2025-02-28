use gpiod;
use gpiod::Options;
use std::error::Error;

/// PinSetup class for configuring GPIO pins for sensor selection.
///
/// # Arguments
///
/// * `sensor_option` - Integer value 0 to 5, where:
///   * 0: None - No sensors active
///   * 1: Set GPIO pin 5 to output mode and drive it low
///   * 2-4: Reserved for additional sensors (currently todo)
///   * 5: Reserved for future use
#[derive(Debug)]
pub struct PinSetup {
    sensor_option: u8,
}

impl PinSetup {
    /// Creates a new PinSetup instance with the specified sensor option
    pub fn new(sensor_option: u8) -> Result<Self, Box<dyn Error>> {
        if sensor_option > 5 {
            return Err(format!("Invalid sensor option: {}. Must be 0-5", sensor_option).into());
        }

        let pin_setup = PinSetup { sensor_option };

        // Apply the pin configuration based on the selected option
        pin_setup.configure()?;

        Ok(pin_setup)
    }

    /// Configures the GPIO pins based on the sensor option
    fn configure(&self) -> Result<(), Box<dyn Error>> {
        let chip = gpiod::Chip::new("gpiochip0")?;
        // FIXME: Implement pin configuration pins
        let opts = Options::output([5]);
        let outputs = chip.request_lines(opts)?;

        match self.sensor_option {
            0 => {
                // No action needed for option 0 (no sensors)
                println!("No sensors selected");
                Ok(())
            }
            1 => {
                outputs.set_values([true, false])?;
                Ok(())
            }
            2 => {
                // TODO: Configure for sensor option 2
                todo!("Implement configuration for sensor option 2")
            }
            3 => {
                // TODO: Configure for sensor option 3
                todo!("Implement configuration for sensor option 3")
            }
            4 => {
                // TODO: Configure for sensor option 4
                todo!("Implement configuration for sensor option 4")
            }
            5 => {
                // TODO: Configure for future use
                todo!("Implement configuration for future use option 5")
            }
            _ => {
                // This should never happen because we check in new()
                Err("Invalid sensor option".into())
            }
        }
    }

    /// Returns the current sensor option
    pub fn get_sensor_option(&self) -> u8 {
        self.sensor_option
    }
}
