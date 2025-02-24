use crate::{Error, SensorInterface, TimestampedDistance};
use gpiod::Options;
use std::collections::VecDeque;
use std::os::unix::io::RawFd;
use std::sync::mpsc::{sync_channel, SyncSender};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::SystemTime;

pub struct InterruptHandler {
    thread: Option<JoinHandle<()>>,
    stop_tx: SyncSender<()>,
    readings_queue: Arc<Mutex<VecDeque<TimestampedDistance>>>,
    event_fd: RawFd,
}

impl InterruptHandler {
    pub fn new(
        sensor: Arc<Mutex<impl SensorInterface + 'static>>,
        chip: &str,
        line: u32,
        queue_size: usize,
    ) -> Result<Self, Error> {
        let (stop_tx, stop_rx) = sync_channel(1);
        let readings_queue = Arc::new(Mutex::new(VecDeque::with_capacity(queue_size)));

        // Open GPIO chip
        let chip = gpiod::Chip::new(chip).map_err(Error::Gpio)?;

        // Configure GPIO line for input with event detection
        let options = Options::input([line])
            .bias(gpiod::Bias::PullDown)
            .edge(gpiod::EdgeDetect::Rising);
        // .build();

        // Request the line
        let mut line_handle = chip.request_lines(options).map_err(Error::Gpio)?;

        // Create event fd
        let event_fd = unsafe { libc::eventfd(0, libc::EFD_NONBLOCK) };
        if event_fd < 0 {
            return Err(Error::EventFd(std::io::Error::last_os_error()));
        }

        let queue = readings_queue.clone();
        let event_fd_clone = event_fd;

        let thread = thread::spawn(move || {
            // Set realtime priority if possible
            #[cfg(target_os = "linux")]
            if let Err(e) = set_realtime_priority() {
                eprintln!("Failed to set thread priority: {}", e);
            }

            while stop_rx.try_recv().is_err() {
                // Check for edge events
                if let Ok(events) = line_handle.read_event() {
                    if !events.edge.eq(&gpiod::Edge::Rising) {
                        let timestamp = SystemTime::now();

                        // Get distance reading
                        if let Ok(distance) = sensor.lock().unwrap().get_distance() {
                            // Create reading
                            let reading = TimestampedDistance {
                                timestamp,
                                distance,
                            };

                            // Clear interrupt for next measurement
                            if let Err(e) = sensor.lock().unwrap().clear_interrupt() {
                                eprintln!("Failed to clear interrupt: {}", e);
                                continue;
                            }

                            // Update queue
                            let mut queue = queue.lock().unwrap();
                            if queue.len() == queue.capacity() {
                                queue.pop_front(); // Remove oldest if full
                                eprintln!("Queue full, dropping oldest reading");
                            }
                            queue.push_back(reading);

                            // Notify Python side
                            let value: u64 = 1;
                            unsafe {
                                libc::write(
                                    event_fd_clone,
                                    &value as *const u64 as *const libc::c_void,
                                    std::mem::size_of::<u64>(),
                                );
                            }
                        }
                    }
                }

                // Small sleep to prevent busy waiting
                std::thread::sleep(std::time::Duration::from_millis(1));
            }
        });

        Ok(InterruptHandler {
            thread: Some(thread),
            stop_tx,
            readings_queue,
            event_fd,
        })
    }

    pub fn get_fd(&self) -> RawFd {
        self.event_fd
    }

    pub fn get_reading(&self) -> Option<TimestampedDistance> {
        let mut queue = self.readings_queue.lock().unwrap();
        queue.pop_front()
    }
}

impl Drop for InterruptHandler {
    fn drop(&mut self) {
        self.stop_tx.send(()).ok();
        if let Some(thread) = self.thread.take() {
            thread.join().ok();
        }
        unsafe {
            libc::close(self.event_fd);
        }
    }
}

#[cfg(target_os = "linux")]
fn set_realtime_priority() -> std::io::Result<()> {
    unsafe {
        let mut sched_param: libc::sched_param = std::mem::zeroed();
        sched_param.sched_priority = 99; // Max RT priority for SCHED_FIFO
        if libc::sched_setscheduler(0, libc::SCHED_FIFO, &sched_param) != 0 {
            return Err(std::io::Error::last_os_error());
        }
    }
    Ok(())
}
