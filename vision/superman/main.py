#!/usr/bin/env python3

import asyncio
import logging
import subprocess
import time
from pathlib import Path

import ntcore

# NetworkTables topic constants
TOPIC_CAM_RESET = "/Pi/cam_reset"
TOPIC_TOF_RESET = "/Pi/tof_reset"
TOPIC_PI_REBOOT = "/Pi/reboot"
TOPIC_FMS_ATTACHED = "/AdvantageKit/DriverStation/FMSAttached"

# Test mode topic constants
TOPIC_STATUS = "/Pi/superman/status"
TOPIC_WIFI_STATUS = "/Pi/superman/wifi_status"
TOPIC_LED_STATUS = "/Pi/superman/led_status"
TOPIC_WIFI_TIMER = "/Pi/superman/wifi_timer"
TOPIC_CAM_RESET_COUNT = "/Pi/superman/cam_reset_count"
TOPIC_TOF_RESET_COUNT = "/Pi/superman/tof_reset_count"


class AsyncTimer:
    """Asyncio-compatible timer with time tracking and cancellation support"""
    def __init__(self, logger=None):
        self.log = logger or logging.getLogger('AsyncTimer')
        self.task = None
        self.start_time = None
        self.duration = 0
        self.callback = None
        self.name = None
        
    async def start(self, seconds, callback, name=None):
        """Start a timer with the specified duration and callback"""
        # Cancel any existing timer
        if self.is_active():
            self.log.info(f"Cancelling existing timer: {self.name}")
            self.cancel()
            
        self.name = name or "timer"
        self.start_time = time.monotonic()
        self.duration = seconds
        self.callback = callback
        
        self.log.info(f"Starting {self.name} for {seconds}s")
        
        # Create task
        self.task = asyncio.create_task(self._run())
        return self.task
        
    async def _run(self):
        """Run the timer and execute callback when complete"""
        try:
            await asyncio.sleep(self.duration)
            self.log.info(f"{self.name} completed after {self.duration}s")
            if self.callback:
                await self.callback()
        except asyncio.CancelledError:
            self.log.info(f"{self.name} was cancelled")
            raise
        finally:
            # Clear state if we completed normally
            if not self.task.cancelled():
                self.clear()
    
    def get_remaining(self):
        """Get remaining time in seconds, or 0 if timer is not active"""
        if not self.is_active():
            return 0
        elapsed = time.monotonic() - self.start_time
        return max(0, self.duration - elapsed)
    
    def is_active(self):
        """Check if timer is currently active"""
        return self.task is not None and not self.task.done()
    
    def cancel(self):
        """Cancel the timer if active"""
        if self.is_active():
            self.task.cancel()
            self.clear()
            return True
        return False
    
    def clear(self):
        """Clear timer state"""
        self.start_time = None
        self.duration = 0
        self.callback = None


class DummyPublisher:
    """No-op publisher for when we don't need to publish"""
    def set(self, *args, **kwargs):
        pass


class CmdRunner:
    """Execute commands with proper logging and error handling"""
    def __init__(self, logger, test_mode=False, mock=False, status_pub=None):
        self.log = logger
        self.test_mode = test_mode
        self.mock = mock
        self.status_pub = status_pub or DummyPublisher()
        
    async def run(self, cmd, desc, ok_msg=None, err_msg=None, mock_delay=1.0):
        """Execute a command with proper handling for real and test modes"""
        self.log.info(f"Running: {' '.join(cmd)}")
        
        # Update status
        self.status_pub.set(desc)
        
        # In test mode with mock commands, don't actually run
        if self.test_mode and self.mock:
            self.log.info(f"MOCK: Would run {' '.join(cmd)}")
            await asyncio.sleep(mock_delay)
            
            if ok_msg:
                self.status_pub.set(ok_msg)
            return True
        
        try:
            proc = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE
            )
            _, stderr = await proc.communicate()
            
            if proc.returncode != 0:
                self.log.error(f"Command failed with code {proc.returncode}")
                if stderr:
                    self.log.error(f"Error: {stderr.decode().strip()}")
                
                if err_msg:
                    self.status_pub.set(err_msg)
                return False
            else:
                self.log.info("Command executed successfully")
                
                if ok_msg:
                    self.status_pub.set(ok_msg)
                return True
                
        except Exception as e:
            self.log.error(f"Error executing command: {e}")
            
            if err_msg:
                self.status_pub.set(err_msg)
            return False


class LEDControl:
    """Control the Raspberry Pi LED"""
    def __init__(self, logger, led_path="/sys/class/leds/ACT"):
        self.log = logger
        self.led_path = Path(led_path)
        self.orig_trigger = None
        self.orig_brightness = None
        
    async def save_state(self):
        """Save the original LED state"""
        try:
            # Read current trigger
            trigger_path = self.led_path / "trigger"
            if trigger_path.exists():
                with open(trigger_path, 'r') as f:
                    content = f.read()
                    for line in content.split():
                        if line.startswith('[') and line.endswith(']'):
                            self.orig_trigger = line[1:-1]
                            break
            
            # Read current brightness
            brightness_path = self.led_path / "brightness"
            if brightness_path.exists():
                with open(brightness_path, 'r') as f:
                    self.orig_brightness = f.read().strip()
                    
            self.log.info(f"Saved LED state: trigger={self.orig_trigger}")
            return True
        except Exception as e:
            self.log.error(f"Error saving LED state: {e}")
            return False
    
    async def restore_state(self):
        """Restore the original LED state"""
        try:
            if self.orig_trigger:
                with open(self.led_path / "trigger", 'w') as f:
                    f.write(self.orig_trigger)
            
            if self.orig_brightness:
                with open(self.led_path / "brightness", 'w') as f:
                    f.write(self.orig_brightness)
                        
            self.log.info("Restored original LED state")
            return True
        except Exception as e:
            self.log.error(f"Error restoring LED state: {e}")
            return False
    
    async def set_flash(self, on_ms=500, off_ms=500):
        """Set LED to flashing mode"""
        try:
            trigger_path = self.led_path / "trigger"
            if not trigger_path.exists():
                self.log.error("LED trigger path not found")
                return False
                
            # Set LED to timer mode for flashing
            with open(trigger_path, 'w') as f:
                f.write("timer")
                
            # Configure timer delay
            with open(self.led_path / "delay_on", 'w') as f:
                f.write(str(on_ms))
                
            with open(self.led_path / "delay_off", 'w') as f:
                f.write(str(off_ms))
            
            self.log.info(f"Set LED to flash mode")
            return True
        except Exception as e:
            self.log.error(f"Error setting LED to flash: {e}")
            return False
    
    async def set_default(self):
        """Set LED to default mode"""
        try:
            with open(self.led_path / "trigger", 'w') as f:
                f.write("mmc0")  # Default for Pi
            
            self.log.info("Set LED to default mode")
            return True
        except Exception as e:
            self.log.error(f"Error setting LED to default: {e}")
            return False


class WiFiMgr:
    """Manage WiFi radio state"""
    def __init__(self, logger, cmd_runner):
        self.log = logger
        self.cmd = cmd_runner
        self.is_off = False
        self.timer = AsyncTimer(logger)
        
    async def set_state(self, disable=True):
        """Enable or disable WiFi radio"""
        if disable:
            cmd = ["rfkill", "block", "wifi"]
            state = "disabled"
        else:
            cmd = ["rfkill", "unblock", "wifi"]
            state = "enabled"
        
        result = await self.cmd.run(
            cmd,
            f"WiFi being {state}",
            ok_msg=f"WiFi {state}",
            err_msg=f"Failed to set WiFi {state}"
        )
        
        if result:
            cmd = ["nmcli", "con", "up", "Team8089"]
            result = await self.cmd.run(
                cmd,
                f"Connecting to Team8089",
                ok_msg=f"WiFi reconnected (?)",
                err_msg=f"Failed to reconnect to hotspot"
            )
            if result:
                self.is_off = disable
            
        return result
    
    async def schedule_enable(self, delay_sec):
        """Schedule WiFi to be re-enabled after delay"""
        # Simply start a new timer with our enable function as callback
        await self.timer.start(delay_sec, self._wifi_enable, "WiFi re-enable")
    
    async def _wifi_enable(self):
        """Re-enable WiFi (callback for timer)"""
        self.log.info("Re-enabling WiFi")
        await self.set_state(disable=False)
    
    def cancel_enable(self):
        """Cancel any pending re-enable task"""
        return self.timer.cancel()


class TopicMgr:
    """Manage NT topics with consistent error handling"""
    def __init__(self, nt_instance, test_mode=False):
        self.nt = nt_instance
        self.topics = {}
        self.listeners = {}
        self.test_mode = test_mode
        
    def create(self, name, type_name, init_val=None, publish=True):
        """Create a topic with publisher and subscriber"""
        try:
            if type_name == "bool":
                topic = self.nt.getBooleanTopic(name)
                default = False if init_val is None else init_val
            elif type_name == "str":
                topic = self.nt.getStringTopic(name)
                default = "" if init_val is None else init_val
            elif type_name == "int":
                topic = self.nt.getIntegerTopic(name)
                default = 0 if init_val is None else init_val
            else:
                raise ValueError(f"Unknown topic type: {type_name}")
                
            sub = topic.subscribe(default)
            
            # Only create real publisher if needed
            if publish and self.test_mode:
                pub = topic.publish()
                pub.set(default)
            else:
                pub = DummyPublisher()
            
            self.topics[name] = {
                'topic': topic,
                'sub': sub,
                'pub': pub,
                'type': type_name
            }
            
            return sub, pub
        except Exception as e:
            logging.error(f"Error creating topic {name}: {e}")
            return None, None
    
    def add_callback(self, topic_name, callback, event_flags=ntcore.EventFlags.kValueAll):
        """Add a callback function for a topic change"""
        try:
            listener = self.nt.addListener(
                [topic_name],
                event_flags,
                callback
            )
            self.listeners[topic_name] = listener
            return listener
        except Exception as e:
            logging.error(f"Error adding callback for {topic_name}: {e}")
            return None
            
    def get_pub(self, name):
        """Get publisher for a topic"""
        return self.topics.get(name, {}).get('pub', DummyPublisher())
        
    def get_sub(self, name):
        """Get subscriber for a topic"""
        return self.topics.get(name, {}).get('sub')
        
    def remove_all_listeners(self):
        """Remove all registered listeners"""
        for listener in self.listeners.values():
            try:
                self.nt.removeListener(listener)
            except Exception:
                pass
        self.listeners.clear()


class ResetService:
    def __init__(self, args):
        self.args = args
        self.log = logging.getLogger("superman")
        self.running = True
        self.fms_attached = False
        self.event_loop = None
        self.event_queue = asyncio.Queue()
        
        # Set up NetworkTables
        self.nt = ntcore.NetworkTableInstance.getDefault()
        self.topic_mgr = TopicMgr(self.nt, test_mode=args.serve_nt)
        
        # Initialize NT
        self.setup_nt()
        
        # Create other components
        status_pub = self.topic_mgr.get_pub(TOPIC_STATUS)
        self.cmd = CmdRunner(
            self.log, 
            test_mode=self.args.serve_nt,
            mock=self.args.mock_commands,
            status_pub=status_pub
        )
        
        # Create LED controller if enabled
        self.led = LEDControl(self.log) if self.args.led else None
            
        # Create WiFi manager
        self.wifi = WiFiMgr(self.log, self.cmd)
        
        # Set up topic -> handler mapping
        self.handlers = {
            TOPIC_CAM_RESET: self._handle_cam_reset,
            TOPIC_TOF_RESET: self._handle_tof_reset,
            TOPIC_PI_REBOOT: self._handle_reboot,
            TOPIC_FMS_ATTACHED: self._handle_fms_state
        }
    
    def setup_nt(self):
        """Set up NetworkTables and create topics"""
        # Set up NT server or client
        if self.args.serve_nt:
            self.log.info("Starting NetworkTables server for testing")
            self.nt.startServer()
        else:
            self.log.info("Connecting to robot's NetworkTables")
            self.nt.setServerTeam(8089)
            self.nt.startClient4("superman")
        
        # Create required topics
        self.cam_reset_sub, self.cam_reset_pub = self.topic_mgr.create(
            TOPIC_CAM_RESET, "bool", False)
        self.tof_reset_sub, self.tof_reset_pub = self.topic_mgr.create(
            TOPIC_TOF_RESET, "bool", False)
        self.reboot_sub, self.reboot_pub = self.topic_mgr.create(
            TOPIC_PI_REBOOT, "bool", False)
        self.fms_sub, self.fms_pub = self.topic_mgr.create(
            TOPIC_FMS_ATTACHED, "bool", False)
        
        # Create test topics if needed
        if self.args.serve_nt:
            self.create_test_topics()
    
    def create_test_topics(self):
        """Create additional topics for testing purposes"""
        self.log.info("Creating test topics")
        
        _, self.status_pub = self.topic_mgr.create(TOPIC_STATUS, "str", "Initialized")
        _, self.wifi_status_pub = self.topic_mgr.create(TOPIC_WIFI_STATUS, "str", "Enabled")
        _, self.led_status_pub = self.topic_mgr.create(TOPIC_LED_STATUS, "str", "Default")
        _, self.timer_pub = self.topic_mgr.create(TOPIC_WIFI_TIMER, "int", 0)
        _, self.cam_reset_count_pub = self.topic_mgr.create(TOPIC_CAM_RESET_COUNT, "int", 0)
        _, self.tof_reset_count_pub = self.topic_mgr.create(TOPIC_TOF_RESET_COUNT, "int", 0)
    
    def on_topic_change(self, event):
        """Callback for NT topic changes - queues events for async processing"""
        try:
            # Add event to asyncio queue to be processed in main loop
            asyncio.run_coroutine_threadsafe(
                self.event_queue.put(event),
                self.event_loop
            )
        except Exception as e:
            self.log.error(f"Error queueing event: {e}")
    
    def setup_listeners(self):
        """Set up event listeners for topics"""
        for topic in self.handlers.keys():
            self.topic_mgr.add_callback(topic, self.on_topic_change)
        self.log.info("NT event listeners registered")
    
    async def _handle_cam_reset(self, value):
        """Handle camera reset events"""
        if value.getBoolean():
            self.log.info("Camera reset requested")
            await self.reset_service("cam")
            self.cam_reset_pub.set(False)
            
            # Update count in test mode
            if self.args.serve_nt:
                try:
                    count = self.topic_mgr.get_sub(TOPIC_CAM_RESET_COUNT).get()
                    self.topic_mgr.get_pub(TOPIC_CAM_RESET_COUNT).set(count + 1)
                except Exception:
                    pass
    
    async def _handle_tof_reset(self, value):
        """Handle TOF reset events"""
        if value.getBoolean():
            self.log.info("TOF reset requested")
            await self.reset_service("tof")
            self.tof_reset_pub.set(False)
            
            # Update count in test mode
            if self.args.serve_nt:
                try:
                    count = self.topic_mgr.get_sub(TOPIC_TOF_RESET_COUNT).get()
                    self.topic_mgr.get_pub(TOPIC_TOF_RESET_COUNT).set(count + 1)
                except Exception:
                    pass
    
    async def _handle_reboot(self, value):
        """Handle reboot events"""
        if value.getBoolean():
            self.log.info("Pi reboot requested")
            await self.reboot_pi()
            self.reboot_pub.set(False)
    
    async def _handle_fms_state(self, value):
        """Handle FMS state changes"""
        fms = value.getBoolean()
        if fms != self.fms_attached:
            self.fms_attached = fms
            self.log.info(f"FMS attached changed to: {fms}")
            
            if self.args.radio:
                # Handle WiFi state changes
                if fms:
                    # If FMS is attached, cancel any pending re-enable
                    self.wifi.cancel_enable()
                    
                    # Reset timer display in test mode
                    self.topic_mgr.get_pub(TOPIC_WIFI_TIMER).set(0)
                                            
                    # Turn off WiFi immediately
                    await self.wifi.set_state(disable=True)
                else:
                    # Schedule delayed WiFi re-enable
                    await self.wifi.schedule_enable(self.args.on_delay)
            
            # Update LED state based on FMS connection
            if self.led:
                await self.update_led()
                
            # Update status in test mode
            status = f"FMS {'Connected' if fms else 'Disconnected'}"
            self.topic_mgr.get_pub(TOPIC_STATUS).set(status)
    
    async def reset_service(self, service_name):
        """Reset a system service"""
        cmd = ["systemctl", "restart", f"{service_name}.service"]
        return await self.cmd.run(
            cmd,
            f"Restarting {service_name} service",
            ok_msg=f"{service_name.capitalize()} service restarted",
            err_msg=f"Failed to restart {service_name} service"
        )
    
    async def reboot_pi(self):
        """Reboot the pi"""
        cmd = ["systemctl", "reboot"]
        return await self.cmd.run(
            cmd,
            "Rebooting Pi",
            ok_msg="Pi rebooting",
            err_msg="Failed to reboot Pi"
        )
    
    async def update_led(self):
        """Update LED based on current WiFi state"""
        if not self.led:
            return
            
        try:
            if self.wifi.is_off:
                await self.led.set_flash()
                self.topic_mgr.get_pub(TOPIC_LED_STATUS).set("Flashing")
            else:
                await self.led.set_default()
                self.topic_mgr.get_pub(TOPIC_LED_STATUS).set("Default")
        except Exception as e:
            self.log.error(f"Error updating LED: {e}")
    
    async def run(self):
        """Main run loop"""
        self.log.info("Reset service starting")
        self.event_loop = asyncio.get_running_loop()
        
        # Save original LED state if needed
        if self.led:
            await self.led.save_state()
        
        # Set up NT event listeners
        self.setup_listeners()
        
        try:
            # Main loop - process events from queue and update timer
            while self.running:
                # Process events from queue with timeout
                try:
                    # Wait for an event with timeout so we can also update the timer
                    event = await asyncio.wait_for(self.event_queue.get(), 1.0)
                    
                    # Process the event
                    topic = event.data.topic.getName()
                    if topic in self.handlers:
                        try:
                            await self.handlers[topic](event.data.value)
                        except Exception as e:
                            self.log.error(f"Error handling {topic}: {e}")
                    
                    # Mark as done
                    self.event_queue.task_done()
                    
                except asyncio.TimeoutError:
                    # This is expected - lets us check timer below
                    pass
                
                # Update timer in test mode
                if self.args.serve_nt and self.wifi.timer.is_active():
                    remain = int(round(self.wifi.timer.get_remaining()))
                    self.topic_mgr.get_pub(TOPIC_WIFI_TIMER).set(remain)
                
        except Exception as e:
            self.log.error(f"Error in main loop: {e}")
        finally:
            # Cleanup
            self.log.info("Reset service shutting down")
            self.running = False
            self.topic_mgr.remove_all_listeners()
            if self.led:
                await self.led.restore_state()


async def main():
    """Main entry point"""
    args = parse_args()
    
    # Configure logging
    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(
        level=log_level,
        format="%(asctime)s.%(msecs)03d:%(levelname)5s:%(name)s: %(message)s",
        datefmt="%H:%M:%S"
    )
    
    # Create and run the service
    service = ResetService(args)
    await service.run()


def parse_args():
    """Parse command line arguments"""
    import argparse
    parser = argparse.ArgumentParser(description="Supervisory Manager")
    
    parser.add_argument("-d", "--debug", action="store_true", 
                        help="Enable debug logging")
    parser.add_argument("--radio", action="store_true",
                        help="Enable WiFi radio control")
    parser.add_argument("--on-delay", type=int, default=300,
                        help="Delay (seconds) before re-enabling WiFi (default: %(default)s)")
    parser.add_argument("--led", action="store_true",
                        help="Enable LED control to indicate WiFi status")
    parser.add_argument("--serve-nt", action="store_true",
                        help="Serve NetworkTables for testing")
    parser.add_argument("--mock-commands", action="store_true",
                        help="Don't execute actual system commands when testing")
    
    return parser.parse_args()


if __name__ == "__main__":
    asyncio.run(main())
