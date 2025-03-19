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
        self.nt.setServerTeam(8089)
        self.nt.startClient4("superman")
        
        # Create subscribers for reset topics
        cam_reset_topic = self.nt.getBooleanTopic("/Pi/cam_reset")
        self.cam_reset_sub = cam_reset_topic.subscribe(False)
        self.cam_reset_pub = cam_reset_topic.publish()
        
        tof_reset_topic = self.nt.getBooleanTopic("/Pi/tof_reset")
        self.tof_reset_sub = tof_reset_topic.subscribe(False)
        self.tof_reset_pub = tof_reset_topic.publish()
        
        # FMS connection topic
        fms_topic = self.nt.getBooleanTopic("/AdvantageKit/FMSAttached")
        self.fms_sub = fms_topic.subscribe(False)
        
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
                        
                elif topic_name == "/Pi/tof_reset":
                    if event.data.value.getBoolean():
                        self.log.info("TOF reset requested")
                        # Schedule the reset task
                        asyncio.run_coroutine_threadsafe(self.reset_service("tof"), loop)
                        # Reset the flag
                        self.tof_reset_pub.set(False)
                        
                elif topic_name == "/AdvantageKit/FMSAttached":
                    fms_attached = event.data.value.getBoolean()
                    if fms_attached != self.fms_attached:
                        self.fms_attached = fms_attached
                        self.log.info(f"FMS attached state changed to: {fms_attached}")
                        
                        if self.args.radio:
                            # Handle WiFi state changes
                            if fms_attached:
                                # Turn off WiFi immediately
                                asyncio.run_coroutine_threadsafe(self.handle_wifi(disable=True), loop)
                            else:
                                # Schedule delayed WiFi re-enable
                                asyncio.run_coroutine_threadsafe(self.schedule_wifi_reenable(), loop)
                        
                        if self.led_control_enabled:
                            # Update LED state based on FMS connection
                            asyncio.run_coroutine_threadsafe(self.update_led(), loop)
            
            # Small sleep to avoid spinning
            time.sleep(0.05)
    
    async def reset_service(self, service_name):
        """Reset a system service"""
        cmd = ["sudo", "systemctl", "restart", f"{service_name}.service"]
        self.log.info(f"Executing: {' '.join(cmd)}")
        
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
            else:
                self.log.info(f"Service {service_name} restarted successfully")
                
        except Exception as e:
            self.log.error(f"Error restarting service: {e}")
    
    async def handle_wifi(self, disable=True):
        """Enable or disable WiFi radio"""
        if disable:
            cmd = ["sudo", "rfkill", "block", "wifi"]
            state_msg = "disabled"
        else:
            cmd = ["sudo", "rfkill", "unblock", "wifi"]
            state_msg = "enabled"
            
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
            else:
                self.wifi_turned_off = disable
                self.log.info(f"WiFi radio {state_msg}")
                
                # Update LED if enabled
                if self.led_control_enabled:
                    await self.update_led()
                    
        except Exception as e:
            self.log.error(f"Error setting WiFi state: {e}")
    
    async def schedule_wifi_reenable(self):
        """Schedule WiFi to be re-enabled after delay"""
        # Cancel any existing task
        if self.wifi_reenable_task and not self.wifi_reenable_task.done():
            self.log.info("Cancelling previous WiFi re-enable task")
            self.wifi_reenable_task.cancel()
        
        delay_minutes = self.args.on_delay
        self.log.info(f"Scheduling WiFi to be re-enabled after {delay_minutes} minutes")
        
        # Create new task
        self.wifi_reenable_task = asyncio.create_task(self.delayed_wifi_enable(delay_minutes))
    
    async def delayed_wifi_enable(self, delay_minutes):
        """Wait for specified delay then re-enable WiFi"""
        try:
            await asyncio.sleep(delay_minutes * 60)
            self.log.info(f"Delay completed, re-enabling WiFi")
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
        except Exception as e:
            self.log.error(f"Error restoring LED state: {e}")
    
    async def update_led(self):
        """Update LED state based on current system status"""
        if not self.led_control_enabled:
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
            else:
                # Return to default LED behavior
                with open(trigger_path, 'w') as f:
                    f.write("mmc0")  # Default activity trigger for Pi5
                
                self.log.info("Set LED to default mode")
                
        except Exception as e:
            self.log.error(f"Error updating LED state: {e}")

