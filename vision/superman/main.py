import asyncio
import logging
import subprocess
import time
import threading
from pathlib import Path

import ntcore

class ResetService:
    def __init__(self, args):
        self.args = args
        self.log = logging.getLogger("superman")
        self.running = True
        
        # For LED control
        self.led_control_enabled = hasattr(args, 'led') and args.led
        self.led_path = Path("/sys/class/leds/ACT")
        self.original_trigger = None
        self.original_brightness = None
        
        # FMS and WiFi state tracking
        self.fms_attached = False
        self.wifi_turned_off = False
        self.wifi_reenable_task = None
        
        # Set up NetworkTables
        self.nt_setup()
    
    def nt_setup(self):
        """Initialize NetworkTables and set up listeners"""
        self.nt = ntcore.NetworkTableInstance.getDefault()
        
        # Set up NT server or client
        if self.args.serve_nt:
            self.log.info("Starting NetworkTables server for testing")
            self.nt.startServer()
        else:
            self.log.info("Connecting to robot's NetworkTables")
            self.nt.setServerTeam(8089)
            self.nt.startClient4("superman")
        
        # Create subscribers and publishers for reset topics
        cam_reset_topic = self.nt.getBooleanTopic("/Pi/cam_reset")
        self.cam_reset_sub = cam_reset_topic.subscribe(False)
        self.cam_reset_pub = cam_reset_topic.publish()
        self.cam_reset_pub.set(False)
        
        tof_reset_topic = self.nt.getBooleanTopic("/Pi/tof_reset")
        self.tof_reset_sub = tof_reset_topic.subscribe(False)
        self.tof_reset_pub = tof_reset_topic.publish()
        self.tof_reset_pub.set(False)
        
        # FMS connection topic
        fms_topic = self.nt.getBooleanTopic("/AdvantageKit/FMSAttached")
        self.fms_sub = fms_topic.subscribe(False)
        
        if self.args.serve_nt:
            # In server mode, also publish the FMS value for testing
            self.fms_pub = fms_topic.publish()
            self.fms_pub.set(False)
            
            # Create additional test topics if serving NT
            self.create_test_topics()
        
        # Create a poller to get notifications
        self.poller = ntcore.NetworkTableListenerPoller(self.nt)
        self.poller.addListener(
            self.cam_reset_sub, 
            ntcore.EventFlags.kValueAll
        )
        self.poller.addListener(
            self.tof_reset_sub, 
            ntcore.EventFlags.kValueAll
        )
        self.poller.addListener(
            self.fms_sub, 
            ntcore.EventFlags.kValueAll
        )
    
    def create_test_topics(self):
        """Create additional topics for testing purposes"""
        self.log.info("Creating test topics")
        
        # Create status topics to show what's happening
        status_topic = self.nt.getStringTopic("/Pi/superman/status")
        self.status_pub = status_topic.publish()
        self.status_pub.set("Initialized")
        
        wifi_status_topic = self.nt.getStringTopic("/Pi/superman/wifi_status")
        self.wifi_status_pub = wifi_status_topic.publish()
        self.wifi_status_pub.set("Enabled")
        
        # Create topic to show LED state
        led_status_topic = self.nt.getStringTopic("/Pi/superman/led_status")
        self.led_status_pub = led_status_topic.publish()
        self.led_status_pub.set("Default")
        
        # Create timer countdown topic for WiFi re-enable delay
        timer_topic = self.nt.getIntegerTopic("/Pi/superman/wifi_timer")
        self.timer_pub = timer_topic.publish()
        self.timer_pub.set(0)  # time remaining
        
        # Create command count topics to track reset commands
        cam_reset_count_topic = self.nt.getIntegerTopic("/Pi/superman/cam_reset_count")
        self.cam_reset_count_pub = cam_reset_count_topic.publish()
        self.cam_reset_count_pub.set(0)
        
        tof_reset_count_topic = self.nt.getIntegerTopic("/Pi/superman/tof_reset_count")
        self.tof_reset_count_pub = tof_reset_count_topic.publish()
        self.tof_reset_count_pub.set(0)
        
    async def run(self):
        """Main run loop"""
        self.log.info("Reset service starting")
        
        # Save original LED state if needed
        if self.led_control_enabled:
            await self.save_led_state()
        
        # Start NT polling thread
        loop = asyncio.get_event_loop()
        nt_thread = threading.Thread(target=self.nt_listener_thread, args=(loop,), daemon=True)
        nt_thread.start()
        
        try:
            # Main loop - just keep running until stopped
            while self.running:
                # In server mode, periodically update test topics
                if self.args.serve_nt and hasattr(self, 'timer_pub') and self.wifi_reenable_task is not None:
                    if not self.wifi_reenable_task.done() and self.wifi_reenable_start_time is not None:
                        elapsed = time.monotonic() - self.wifi_reenable_start_time
                        self.timer_pub.set(max(int(round(self.wifi_reenable_duration - elapsed)), 0))
                
                await asyncio.sleep(1)
        finally:
            # Cleanup
            self.log.info("Superman service shutting down")
            self.running = False
            if self.led_control_enabled:
                await self.restore_led_state()
    
    def nt_listener_thread(self, loop):
        """Thread that listens for NetworkTable events"""
        self.log.info("NT listener thread started")
       
        while self.running:
            # Poll for events
            events = self.poller.readQueue()
            for event in events:
                # Get topic name
                topic_name = event.data.topic.getName()
                
                # Handle different topics
                if topic_name == "/Pi/cam_reset":
                    if event.data.value.getBoolean():
                        self.log.info("Camera reset requested")
                        # Schedule the reset task
                        asyncio.run_coroutine_threadsafe(self.reset_service("cam"), loop)
                        # Reset the flag
                        self.cam_reset_pub.set(False)
                        
                        # Update count in test mode
                        if self.args.serve_nt and hasattr(self, 'cam_reset_count_pub'):
                            count = ntcore.Value.getInteger(self.nt.getEntry("/Pi/superman/cam_reset_count").getValue())
                            self.cam_reset_count_pub.set(count + 1)
                        
                elif topic_name == "/Pi/tof_reset":
                    if event.data.value.getBoolean():
                        self.log.info("TOF reset requested")
                        # Schedule the reset task
                        asyncio.run_coroutine_threadsafe(self.reset_service("tof"), loop)
                        # Reset the flag
                        self.tof_reset_pub.set(False)
                        
                        # Update count in test mode
                        if self.args.serve_nt and hasattr(self, 'tof_reset_count_pub'):
                            count = ntcore.Value.getInteger(self.nt.getEntry("/Pi/superman/tof_reset_count").getValue())
                            self.tof_reset_count_pub.set(count + 1)
                        
                elif topic_name == "/AdvantageKit/FMSAttached":
                    fms_attached = event.data.value.getBoolean()
                    if fms_attached != self.fms_attached:
                        self.fms_attached = fms_attached
                        self.log.info(f"FMS attached state changed to: {fms_attached}")
                        
                        if self.args.radio:
                            # Handle WiFi state changes
                            if fms_attached:
                                # If FMS is attached again, cancel any pending re-enable timer
                                if self.wifi_reenable_task and not self.wifi_reenable_task.done():
                                    self.log.info("Cancelling WiFi re-enable due to FMS reconnection")
                                    self.wifi_reenable_task.cancel()
                                    self.wifi_reenable_task = None
                                    self.wifi_reenable_start_time = None
    
                                    # Reset timer display in test mode
                                    if self.args.serve_nt and hasattr(self, 'timer_pub'):
                                        self.timer_pub.set(0)
                                                    
                                # Turn off WiFi immediately
                                asyncio.run_coroutine_threadsafe(self.handle_wifi(disable=True), loop)
                            else:
                                # Schedule delayed WiFi re-enable
                                asyncio.run_coroutine_threadsafe(self.schedule_wifi_reenable(), loop)
                        
                        if self.led_control_enabled:
                            # Update LED state based on FMS connection
                            asyncio.run_coroutine_threadsafe(self.update_led(), loop)
                            
                        # Update status in test mode
                        if self.args.serve_nt and hasattr(self, 'status_pub'):
                            self.status_pub.set(f"FMS {'Connected' if fms_attached else 'Disconnected'}")
            
            # Small sleep to avoid spinning
            time.sleep(0.05)
    
    async def reset_service(self, service_name):
        """Reset a system service"""
        cmd = ["systemctl", "restart", f"{service_name}.service"]
        self.log.info(f"Executing: {' '.join(cmd)}")
        
        # Update status in test mode
        if self.args.serve_nt and hasattr(self, 'status_pub'):
            self.status_pub.set(f"Restarting {service_name} service")
        
        # In test mode, don't actually restart services
        if self.args.serve_nt and self.args.mock_commands:
            self.log.info(f"MOCK: Would restart {service_name} service")
            await asyncio.sleep(1)  # Simulate service restart time
            
            if self.args.serve_nt and hasattr(self, 'status_pub'):
                self.status_pub.set(f"{service_name.capitalize()} service restarted")
            return
        
        try:
            proc = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE
            )
            stdout, stderr = await proc.communicate()
            
            if proc.returncode != 0:
                self.log.error(f"Service restart failed with code {proc.returncode}")
                if stderr:
                    self.log.error(f"Error: {stderr.decode().strip()}")
                
                # Update status in test mode
                if self.args.serve_nt and hasattr(self, 'status_pub'):
                    self.status_pub.set(f"Failed to restart {service_name} service")
            else:
                self.log.info(f"Service {service_name} restarted successfully")
                
                # Update status in test mode
                if self.args.serve_nt and hasattr(self, 'status_pub'):
                    self.status_pub.set(f"{service_name.capitalize()} service restarted")
                
        except Exception as e:
            self.log.error(f"Error restarting service: {e}")
            
            # Update status in test mode
            if self.args.serve_nt and hasattr(self, 'status_pub'):
                self.status_pub.set(f"Error restarting {service_name} service")
    
    async def handle_wifi(self, disable=True):
        """Enable or disable WiFi radio"""
        if disable:
            cmd = ["rfkill", "block", "wifi"]
            state_msg = "disabled"
        else:
            cmd = ["rfkill", "unblock", "wifi"]
            state_msg = "enabled"
        
        # Update status in test mode
        if self.args.serve_nt:
            if hasattr(self, 'status_pub'):
                self.status_pub.set(f"WiFi being {state_msg}")
            if hasattr(self, 'wifi_status_pub'):
                self.wifi_status_pub.set(state_msg.capitalize())
        
        # In test mode, don't actually change WiFi state
        if self.args.serve_nt and self.args.mock_commands:
            self.log.info(f"MOCK: Would set WiFi to {state_msg}")
            self.wifi_turned_off = disable
            
            # Update LED if enabled
            if self.led_control_enabled:
                await self.update_led()
            return
            
        try:
            self.log.info(f"Setting WiFi radio {state_msg}")
            proc = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE
            )
            stdout, stderr = await proc.communicate()
            
            if proc.returncode != 0:
                self.log.error(f"WiFi {state_msg} failed with code {proc.returncode}")
                if stderr:
                    self.log.error(f"Error: {stderr.decode().strip()}")
                
                # Update status in test mode
                if self.args.serve_nt and hasattr(self, 'status_pub'):
                    self.status_pub.set(f"Failed to set WiFi {state_msg}")
            else:
                self.wifi_turned_off = disable
                self.log.info(f"WiFi radio {state_msg}")
                
                # Update status in test mode
                if self.args.serve_nt and hasattr(self, 'status_pub'):
                    self.status_pub.set(f"WiFi {state_msg}")
                
                # Update LED if enabled
                if self.led_control_enabled:
                    await self.update_led()
                    
        except Exception as e:
            self.log.error(f"Error setting WiFi state: {e}")
            
            # Update status in test mode
            if self.args.serve_nt and hasattr(self, 'status_pub'):
                self.status_pub.set(f"Error setting WiFi {state_msg}")
    
    async def schedule_wifi_reenable(self):
        """Schedule WiFi to be re-enabled after delay"""
        # Cancel any existing task
        if self.wifi_reenable_task and not self.wifi_reenable_task.done():
            self.log.info("Cancelling previous WiFi re-enable task")
            self.wifi_reenable_task.cancel()
        
        delay_sec = self.args.on_delay
        self.log.info(f"Scheduling WiFi to be re-enabled after {delay_sec} minutes")

        # Store start time and duration for tracking
        self.wifi_reenable_start_time = time.monotonic()
        self.wifi_reenable_duration = delay_sec
        
        # Update status in test mode
        if self.args.serve_nt: 
            if hasattr(self, 'status_pub'):
                self.status_pub.set(f"WiFi re-enable scheduled in {delay_sec} minutes")
            if hasattr(self, 'timer_pub'):
                self.timer_pub.set(delay_sec)
        
        # Create new task
        self.wifi_reenable_task = asyncio.create_task(self.delayed_wifi_enable(delay_sec))
    
    async def delayed_wifi_enable(self, delay_sec):
        """Wait for specified delay then re-enable WiFi"""
        try:
            await asyncio.sleep(delay_sec)
            self.log.info(f"Delay completed, re-enabling WiFi")
            
            # Update status in test mode
            if self.args.serve_nt and hasattr(self, 'status_pub'):
                self.status_pub.set(f"WiFi re-enable timer completed")
            
            if hasattr(self, 'timer_pub'):
                self.timer_pub.set(0)

            await self.handle_wifi(disable=False)
        except asyncio.CancelledError:
            self.log.info("WiFi re-enable task was cancelled")
    
    async def save_led_state(self):
        """Save the original LED state"""
        try:
            # Read current trigger
            trigger_path = self.led_path / "trigger"
            if trigger_path.exists():
                with open(trigger_path, 'r') as f:
                    content = f.read()
                    # The current trigger is marked with [brackets]
                    for line in content.split():
                        if line.startswith('[') and line.endswith(']'):
                            self.original_trigger = line[1:-1]
                            break
            
            # Read current brightness
            brightness_path = self.led_path / "brightness"
            if brightness_path.exists():
                with open(brightness_path, 'r') as f:
                    self.original_brightness = f.read().strip()
                    
            self.log.info(f"Saved original LED state: trigger={self.original_trigger}, brightness={self.original_brightness}")
        except Exception as e:
            self.log.error(f"Error saving LED state: {e}")
    
    async def restore_led_state(self):
        """Restore the original LED state"""
        try:
            if self.original_trigger:
                trigger_path = self.led_path / "trigger"
                if trigger_path.exists():
                    with open(trigger_path, 'w') as f:
                        f.write(self.original_trigger)
            
            if self.original_brightness:
                brightness_path = self.led_path / "brightness"
                if brightness_path.exists():
                    with open(brightness_path, 'w') as f:
                        f.write(self.original_brightness)
                        
            self.log.info("Restored original LED state")
            
            # Update status in test mode
            if self.args.serve_nt and hasattr(self, 'led_status_pub'):
                self.led_status_pub.set("Original")
        except Exception as e:
            self.log.error(f"Error restoring LED state: {e}")
    
    async def update_led(self):
        """Update LED state based on current system status"""
        if not self.led_control_enabled:
            return
        
        # In test mode, just update status but don't touch LED
        if self.args.serve_nt and self.args.mock_commands:
            if hasattr(self, 'led_status_pub'):
                self.led_status_pub.set("Flashing" if self.wifi_turned_off else "Default")
            return
            
        try:
            trigger_path = self.led_path / "trigger"
            if not trigger_path.exists():
                self.log.error("LED trigger path does not exist")
                return
                
            # If WiFi is off (due to FMS), set flashing LED
            if self.wifi_turned_off:
                # Set LED to timer mode for flashing
                with open(trigger_path, 'w') as f:
                    f.write("timer")
                    
                # Configure timer delay (in ms)
                delay_on_path = self.led_path / "delay_on"
                if delay_on_path.exists():
                    with open(delay_on_path, 'w') as f:
                        f.write("500")  # 500ms on
                        
                delay_off_path = self.led_path / "delay_off"
                if delay_off_path.exists():
                    with open(delay_off_path, 'w') as f:
                        f.write("500")  # 500ms off
                
                self.log.info("Set LED to flash mode (WiFi disabled)")
                
                # Update status in test mode
                if self.args.serve_nt and hasattr(self, 'led_status_pub'):
                    self.led_status_pub.set("Flashing")
            else:
                # Return to default LED behavior
                with open(trigger_path, 'w') as f:
                    f.write("mmc0")  # Default activity trigger for Pi5
                
                self.log.info("Set LED to default mode")
                
                # Update status in test mode
                if self.args.serve_nt and hasattr(self, 'led_status_pub'):
                    self.led_status_pub.set("Default")
                
        except Exception as e:
            self.log.error(f"Error updating LED state: {e}")

