use std::time::SystemTime;

#[derive(Debug, Clone)]
pub struct TimestampedDistance {
    pub timestamp: SystemTime,
    pub distance: u16,
}

#[derive(Debug)]
pub enum Error {
    Gpio(std::io::Error),
    ThreadPriority(thread_priority::Error),
    EventFd(std::io::Error),
    SensorComm(String),
    QueueFull,
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::Gpio(e) => write!(f, "GPIO error: {}", e),
            Error::ThreadPriority(e) => write!(f, "Thread priority error: {}", e),
            Error::EventFd(e) => write!(f, "EventFd error: {}", e),
            Error::SensorComm(e) => write!(f, "Sensor communication error: {}", e),
            Error::QueueFull => write!(f, "Reading queue is full"),
        }
    }
}

impl std::error::Error for Error {}
