pub trait SensorInterface: Send + Sync {
    fn get_distance(&self) -> Result<u16, Box<dyn std::error::Error>>;
    fn clear_interrupt(&self) -> Result<(), Box<dyn std::error::Error>>;
}
