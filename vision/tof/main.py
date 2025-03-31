#!/usr/bin/env python3
'''TOF Sensor reader'''

import asyncio
import datetime
import logging
from pathlib import Path
import re
import sys
import signal
import threading
import time

import ntcore

from .cd3 import CornerDetector
from .logs import LogManager, Name, LOG_DIR
from .sensor import SensorManager, SetPins

__version__ = "0.4"  # Updated version

# Constants for NT disconnection handling
DISCONNECTION_GRACE_PERIOD = 5.0  # seconds to wait before considering truly disconnected
FMS_CONTEXT_WAIT_PERIOD = 2.0     # seconds to wait after FMS attaches to fetch context


class TofMain:
    def __init__(self, args):
        self.args = args
        self.log = logging.getLogger("tof")
        self.running = False
        self.speed = self.args.speed
        self.tof_mode = 'none'
        self.saw_corner = False
        self.chute_mode = 'home/right'
        self.chute_side = 'right'
        
        # Network state tracking
        self.rio_connected = False
        self.disconnect_time = None
        self.fms_attached = False
        self.fms_attached_time = None
        self.match_context_pending = False

        # Robot state tracking
        self.robot_enabled = False
        
        # Asyncio components
        self.loop = None
        self.events = asyncio.Queue()
        self.pending_timeouts = {}
        
        # time-related values
        self._ts_base = time.monotonic() 

        # Initialize NetworkTables
        self.nt = None

    def zerotime(self, ts=None):
        '''Make (or convert) a mono timestamp relative to our start time.'''
        if ts is None:
            ts = time.monotonic()
        return ts - self._ts_base

    def nt_setup(self):
        '''initialize NetworkTables stuff'''
        self.nt = ntcore.NetworkTableInstance.getDefault()
        
        if self.args.serve:
            self.log.info('starting test NT server')
            self.nt.startServer()
        else:
            self.nt.setServerTeam(8089)
            self.nt.startClient4("tof")
        
        # shortcut to add and remember listeners
        def add(*args, **kwargs):
            x = self.nt.addListener(*args, **kwargs)
            self.nt_listeners.append(x)

        # Add connection listener
        x = self.nt.addConnectionListener(immediate_notify=True, callback=self.on_connection_change)
        self.nt_listeners = [x]
        
        # Create subscriber for tof_mode
        tof_mode_topic = self.nt.getStringTopic(Name.TOF_MODE)
        self.tof_mode_sub = tof_mode_topic.subscribe('none',
           ntcore.PubSubOptions(keepDuplicates=True))
        add(self.tof_mode_sub, ntcore.EventFlags.kValueAll, self.on_tof_mode_change)
        
        # Create subscriber for chute_mode
        chute_mode_topic = self.nt.getStringTopic(Name.CHUTE_MODE)
        self.chute_mode_sub = chute_mode_topic.subscribe('none')
        add(self.chute_mode_sub, ntcore.EventFlags.kValueAll, self.on_chute_mode_change)
        
        # Match state topics
        fms_topic = self.nt.getBooleanTopic(Name.FMS_ATTACHED)
        self.fms_sub = fms_topic.subscribe(False)
        add(self.fms_sub, ntcore.EventFlags.kValueAll, self.on_fms_change)
        
        topic = self.nt.getBooleanTopic(Name.ENABLED)
        self.enabled_sub = topic.subscribe(False)
        add(self.enabled_sub, ntcore.EventFlags.kValueAll, self.on_enabled_change)
        
        # Match info topics (for log naming)
        mtype_topic = self.nt.getIntegerTopic(Name.MATCH_TYPE)
        self.match_type_sub = mtype_topic.subscribe(0)
        add(self.match_type_sub, ntcore.EventFlags.kValueAll, self.on_match_type_change)
        
        mnum_topic = self.nt.getIntegerTopic(Name.MATCH_NUMBER)
        self.match_num_sub = mnum_topic.subscribe(0)
        add(self.match_num_sub, ntcore.EventFlags.kValueAll, self.on_match_number_change)
        
        ename_topic = self.nt.getStringTopic(Name.EVENT_NAME)
        self.event_name_sub = ename_topic.subscribe("")
        add(self.event_name_sub, ntcore.EventFlags.kValueAll, self.on_event_name_change)
        
        topic = self.nt.getIntegerTopic(Name.REPLAY_NUMBER)
        self.replay_num_sub = topic.subscribe(0)
        add(self.replay_num_sub, ntcore.EventFlags.kValueAll, self.on_replay_number_change)
        
        # Publishers
        self.corner_ts_pub = self.nt.getFloatTopic(Name.CORNER_TS).publish()
        self.corner_ts_pub.set(0.0)
        
        self.corner_dist_pub = self.nt.getFloatTopic(Name.CORNER_DIST_MM).publish()
        self.corner_dist_pub.set(0.0)
        
        self.corner_pub = self.nt.getFloatArrayTopic(Name.CORNERS).publish()
        self.corner_pub.set([0.0, 0.0])
        
        self.ts_dist_pub = self.nt.getDoubleArrayTopic(Name.TS_DIST_MM).publish()
        self.ts_dist_pub.set([0, 0])
        
        self.dist_pub = self.nt.getFloatTopic(Name.DIST_MM).publish()
        self.dist_pub.set(0)
        
        # Set publishers for chute/tof mode (in case we're serving for testing)
        if self.args.serve:
            self.tof_mode_pub = tof_mode_topic.publish()
            self.tof_mode_pub.set('none')
            self.chute_mode_pub = chute_mode_topic.publish()
            self.chute_mode_pub.set('home/right')
            self.fms_pub = fms_topic.publish()
            self.fms_pub.set(False)
            self.mtype_pub = mtype_topic.publish()
            self.mtype_pub.set(2)
            self.mnum_pub = mnum_topic.publish()
            self.mnum_pub.set(31)
            self.ename_pub = ename_topic.publish()
            self.ename_pub.set('onwat')
        
        # Brief pause, hoping beyond hope that NT will compare clocks and
        # set the offset during this.
        # Note: empirically measured it takes only about 5-10 ms for it to
        # measure the offset after starting the client, so this wait is adequate.
        # time.sleep(0.2)

    def nt_shutdown(self):
        for x in self.nt_listeners:
            self.nt.removeListener(x)

    # NT Event handlers - these queue events for the main asyncio loop
    def on_connection_change(self, event):
        """Handle NT connection state changes"""
        connected = bool(event.flags & 0x02)
        self.log.debug("Connection event: %s (flags 0x%02x)", connected, event.flags)
        self.loop.call_soon_threadsafe(self.events.put_nowait,
            ('connection', connected))
    
    def on_tof_mode_change(self, event):
        """Handle tof_mode changes"""
        value = event.data.value.getString()
        # self.log.debug(f"TOF mode event: {value}")
        self.loop.call_soon_threadsafe(self.events.put_nowait,
            ('tof_mode', value))
    
    def on_chute_mode_change(self, event):
        """Handle chute_mode changes"""
        value = event.data.value.getString()
        # self.log.debug(f"Chute mode event: {value}")
        self.loop.call_soon_threadsafe(self.events.put_nowait,
            ('chute_mode', value))
    
    def on_fms_change(self, event):
        """Handle FMS attachment state changes"""
        value = event.data.value.getBoolean()
        self.log.debug(f"FMS attached event: {value}")
        self.loop.call_soon_threadsafe(self.events.put_nowait,
            ('fms_attached', value))
    
    def on_enabled_change(self, event):
        """Handle robot enabled state changes"""
        value = event.data.value.getBoolean()
        # self.log.debug(f"Enabled event: {value}")
        self.loop.call_soon_threadsafe(self.events.put_nowait,
            ('enabled', value))
    
    def on_match_type_change(self, event):
        """Handle match type changes"""
        value = event.data.value.getInteger()
        # self.log.debug(f"Match type event: {value}")
        self.loop.call_soon_threadsafe(self.events.put_nowait,
            ('match_type', value))
    
    def on_match_number_change(self, event):
        """Handle match number changes"""
        value = event.data.value.getInteger()
        # self.log.debug(f"Match number event: {value}")
        self.loop.call_soon_threadsafe(self.events.put_nowait,
            ('match_number', value))
    
    def on_event_name_change(self, event):
        """Handle event name changes"""
        value = event.data.value.getString()
        # self.log.debug(f"Event name event: {value}")
        self.loop.call_soon_threadsafe(self.events.put_nowait,
            ('event_name', value))
    
    def on_replay_number_change(self, event):
        """Handle replay number changes"""
        value = event.data.value.getInteger()
        # self.log.debug(f"Replay number event: {value}")
        self.loop.call_soon_threadsafe(self.events.put_nowait,
            ('replay_number', value))

    def mono_to_fpga(self, mono):
        '''Convert a mono timestamp (including from Rust Instant::now()
        to an FPGA timestamp using the latest server time offset. Note
        that the server time offset currently does NOT get updated
        after the first time so our clocks will drift relative to each other.
        Measurements show with the current Rio and Pi5 that the drift
        is only about 5ppm, which amounts to roughly 1.5ms during a 5 minute
        which at 450mm/s would be less than 1mm of travel, so safe for now.'''
        # Rust will use Instant::now() which is time.monotonic(),
        # while ntcore._now() is supposedly the monotonic clock
        # but has a value like an epoch time, so it appears they initialize
        # it with the offset between epoch time and time.monotonic(),
        # so we need to get that back so we can convert Rust times to
        # ntcore times so that getServerTimeOffset() can be added to
        # convert the times to FPGATimestamp values... sigh.
        # Note: repeated connections to the robot will produce server time
        # offset values that vary up and down within a roughly +/-800us
        # range, so our overall accuracy on any given connection is about
        # that much.
        st_offset = self.nt.getServerTimeOffset()
        # When we're not connected this will return None so callers must handle that.
        if st_offset is None:
            return None
            
        # TODO: We need to calculate ntcore_epoch here. Could be stored as class variable during setup
        # For now, use a best-guess approach (as a simplification)
        ntcore_epoch = ntcore._now() / 1e6 - time.monotonic()
        return mono + ntcore_epoch + st_offset / 1e6

    def on_reading(self, ts, dist_mm, status, delta):
        """Process a TOF reading - may log and publish data"""
        # # Check for log cycling - if we're not in corner mode
        # self.log_manager.check_cycle_timeout(self.tof_mode)
        
        # Log the distance reading
        self.log_manager.append_double(Name.DIST_MM, dist_mm, ts)
        
        flush = False # whether to flush NT (any time we publish)
        cd = self.cd
        cd.add_record(ts, dist_mm, self.speed)
        if cd.found_corner():
            self.saw_corner = True
            corner_ts_fpga = self.mono_to_fpga(cd.corner_timestamp)
            if corner_ts_fpga is not None:
                # Log corner detection with both timestamps
                self.log_manager.append_double(Name.CORNER, cd.corner_dist, cd.corner_timestamp)
                self.log_manager.append_double(Name.CORNER, 0, ts)
            
                # Convert corner mono time to FPGA for the robot and send via NT
                self.corner_pub.set([corner_ts_fpga, cd.corner_dist])
                self.corner_ts_pub.set(cd.corner_timestamp)
                self.corner_dist_pub.set(cd.corner_dist)
                flush = True

                self.log.info("CORNER: @%.3f,%.3f,%.3f,%.3fs", cd.corner_timestamp,
                     ts, corner_ts_fpga, self.speed)

            cd.log_timing()
            cd.reset()
        else:
            if self.args.stdout:
                print("dist,%8.3f,%5.0f,%2d,%5.3f      " % (ts, dist_mm, status, delta), end='\r')

        if self.tof_mode == 'corner':
            ts_fpga = self.mono_to_fpga(ts)
            if ts_fpga is not None:
                # Publish to NT
                self.ts_dist_pub.set([ts_fpga, dist_mm])

                if self.saw_corner:
                    flush = True

        self.dist_pub.set(dist_mm)
        if flush:
            self.nt.flush()

    # Map of modes to GPIO pin indices [5, 14]
    MODE_MAP = {
        'none': 0, # temporarily map to right so we keep logging something all the time
        'left': 1,
        'right': 0,
        'front-left': 1,
        'front-right': 0,
        'rear-left': 1,
        'rear-right': 0,
    }

    def launch_sensor_thread(self):
        """Task that runs the TOF sensor reader in a separate thread"""
        def reader():
            self.mgr.read(self.args.timing, self.args.inter, roi=self.args.roi,
                callback=self.on_reading)

        # Start the reader in a thread
        thread = threading.Thread(target=reader, daemon=True)
        thread.start()
        self.log.debug('started sensor thread')
            
    async def handle_connection_event(self, connected):
        """Handle connection state changes with debouncing"""
        now = time.monotonic()
        
        if connected:
            # Rio is connected (or reconnected)
            if not self.rio_connected:
                self.rio_connected = True
                self.disconnect_time = None
                self.log.info("Rio connected - NT communication established")
                
                # If we were disconnected, start a new log
                if not self.log_manager.awaiting_match_context:
                    self.log.info("Starting new log after connection")
                    self.log_manager.start_new_log()
        else:
            # Rio is disconnected
            if self.rio_connected:
                # Start disconnection grace period
                if self.disconnect_time is None:
                    self.disconnect_time = now
                    self.log.info(f"Rio disconnection detected, starting {DISCONNECTION_GRACE_PERIOD:.1f}s grace period")
                    
                    # Schedule a check after the grace period
                    self.pending_timeouts['disconnect'] = asyncio.create_task(
                        self.check_disconnection_timeout()
                    )

    async def handle_enabled_event(self, enabled):
        """Handle robot enabled state changes"""
        # Log the enabled state change (for information/debugging)
        self.log.debug(f"Robot enabled state: {enabled}")

        # Currently we don't need to take any specific action on enabled state changes
        # If we wanted to use this as another log cycling signal, we could add logic like:

        # if enabled and self.tof_mode != "corner" and not self.log_manager.awaiting_match_context:
        #     # Start a new log when robot is enabled and not in corner mode
        #     # Only if we're not already waiting for match context or in corner mode
        #     if time.monotonic() - self.last_enabled_time > MIN_ENABLE_INTERVAL:
        #         self.log.info("Starting new log due to robot being enabled")
        #         self.log_manager.start_new_log()
        # 
        # self.last_enabled_time = time.monotonic()

        # For now, we just track the state but don't take action
        self.robot_enabled = enabled

    async def check_disconnection_timeout(self):
        """Check if disconnection has persisted past grace period"""
        try:
            await asyncio.sleep(DISCONNECTION_GRACE_PERIOD)
            
            # If we're still disconnected after the grace period
            if self.disconnect_time is not None:
                self.log.info("Rio disconnection confirmed after grace period")
                self.rio_connected = False
                
                # Clear FMS state since we're disconnected
                if self.fms_attached:
                    self.fms_attached = False
                    self.log_manager.set_fms_attached(False)
        except asyncio.CancelledError:
            # Task was cancelled, disconnection was just temporary
            pass
        finally:
            # Clear the pending timeout
            self.pending_timeouts.pop('disconnect', None)
    
    async def handle_fms_event(self, attached):
        """Handle FMS attachment state changes"""
        if attached == self.fms_attached:
            return  # No change
            
        self.fms_attached = attached
        self.log_manager.set_fms_attached(attached)
        
        if attached:
            # FMS just attached - schedule match context collection
            self.fms_attached_time = time.monotonic()
            self.match_context_pending = True
            
            # Schedule context collection after a brief delay
            self.pending_timeouts['fms_context'] = asyncio.create_task(
                self.collect_match_context()
            )
    
    async def collect_match_context(self):
        """Collect match context data after FMS connection"""
        try:
            # Wait briefly for all match data to propagate
            await asyncio.sleep(FMS_CONTEXT_WAIT_PERIOD)
            
            # Collect match context data
            event_name = self.event_name_sub.get()
            match_type = self.match_type_sub.get()
            match_number = self.match_num_sub.get()
            replay_number = self.replay_num_sub.get()
            
            self.log.info(f"Collected match context: {event_name} {match_type} {match_number} {replay_number}")
            
            # Update log manager with context data
            self.log_manager.update_match_context(
                event_name=event_name,
                match_type=match_type,
                match_number=match_number,
                replay_number=replay_number
            )
        except asyncio.CancelledError:
            # Task was cancelled
            pass
        finally:
            # Clear pending flags and tasks
            self.match_context_pending = False
            self.pending_timeouts.pop('fms_context', None)
    
    async def handle_tof_mode_event(self, mode):
        """Handle TOF mode changes"""
        if mode == self.tof_mode:
            return  # No change
            
        # Log the mode change
        self.log_manager.append_string(Name.TOF_MODE, mode, self.zerotime())
        
        # Clear any corner state when mode changes
        self.ts_dist_pub.set([0, 0])
        self.dist_pub.set(0)
        self.saw_corner = False
        self.tof_mode = mode
        self.log.info(f'TOF mode: {mode}')
    
    async def handle_chute_mode_event(self, mode):
        """Handle chute mode changes"""
        if mode == self.chute_mode:
            return  # No change
            
        # Log the mode change
        self.log_manager.append_string(Name.CHUTE_MODE, mode, self.zerotime())
        
        self.chute_mode = mode
        self.log.info(f'Chute mode: {mode}')
        
        # Parse the position/side from the mode
        pos, _, side = mode.partition('/')
        if pos != 'load':
            side = 'left' if side == 'right' else 'right'
            
        if self.chute_side != side:
            self.chute_side = side
            
            # Handle mode changes - switch the pins
            self.pins.set_index_high(None)
            # This gives enough time for the thread to attempt a reading
            # and get an i2c error because the sensor will be offline
            await asyncio.sleep(0.1)
            self.pins.set_index_high(self.MODE_MAP.get(self.chute_side))
            self.log.info(f'Selected TOF: {self.chute_side} (from {mode})')
            
            # Reset corner detector when changing sides
            self.cd.reset()
    
    async def process_event(self, event):
        """Process a queued event based on its type"""
        tag, data = event
        
        try:
            if tag == 'connection':
                await self.handle_connection_event(data)
            elif tag == 'enabled':
                await self.handle_enabled_event(data)
            elif tag == 'fms_attached':
                await self.handle_fms_event(data)
            elif tag == 'tof_mode':
                await self.handle_tof_mode_event(data)
            elif tag == 'chute_mode':
                await self.handle_chute_mode_event(data)
            elif tag == 'match_type':
                if self.match_context_pending:
                    self.log_manager.update_match_context(match_type=data)
            elif tag == 'match_number':
                if self.match_context_pending:
                    self.log_manager.update_match_context(match_number=data)
            elif tag == 'event_name':
                if self.match_context_pending:
                    self.log_manager.update_match_context(event_name=data)
            elif tag == 'replay_number':
                if self.match_context_pending:
                    self.log_manager.update_match_context(replay_number=data)
            else:
                self.log.warning(f"Unknown event type: {tag}")
        except Exception as e:
            self.log.error(f"Error processing {tag} event: {e}")

    def shutdown(self):
        self.log.warning('Shutdown requested')
        self.loop.call_soon_threadsafe(self.terminate.set)
        if self.loop:
            for task in asyncio.all_tasks(self.loop):
                if task is not asyncio.current_task():
                    task.cancel()

    async def wait_terminate(self):
        await self.terminate.wait()
        self.events.put(('terminate', None))

    async def run(self):
        self.loop = asyncio.get_running_loop()

        self.log.info('-' * 40)
        self.log.info('tof v%s', __version__)
        self.log.info('args %r', self.args)

        self.terminate = asyncio.Event()

        def exit_handler(_signal, _frame):
            self.shutdown()
        signal.signal(signal.SIGINT, exit_handler)
        signal.signal(signal.SIGTERM, exit_handler)

        # Initialize log manager
        self.log_manager = LogManager(LOG_DIR, cycle_minutes=self.args.log_cycle_time)
        
        self.nt_setup()

        self.cd = CornerDetector(400)
        self.running = True
        self.pins = SetPins([5, 14])
        self.mgr = SensorManager(self.args.tof_address)

        # Initialize pin for the default chute side
        self.pins.set_index_high(self.MODE_MAP.get(self.chute_side))

        # Start the sensor reader thread
        self.launch_sensor_thread()

        terminate_task = asyncio.create_task(self.wait_terminate())

        try:
            TIMEOUT = 5
            next_check = time.monotonic()
            
            # Main event loop
            while True:
                try:
                    # Wait for an event with timeout to allow for periodic checks
                    try:
                        event = await asyncio.wait_for(self.events.get(), timeout=TIMEOUT)
                    except asyncio.TimeoutError:
                        # self.log.debug('timed out')
                        pass
                    else:
                        # we have a message
                        self.log.debug('event %r', event)
                        await self.process_event(event)
                        self.events.task_done()

                    # Periodic checks that should run regardless of events
                    # Check log cycling needs
                    now = time.monotonic()
                    if now >= next_check:
                        next_check = now + TIMEOUT
                        self.log_manager.check_cycle_timeout(self.tof_mode)
                
                except Exception as e:
                    self.log.error(f"Error in main loop: {e}")
                    
        except asyncio.CancelledError:
            # Main task cancelled
            self.log.info("Main task cancelled")

        finally:
            self.log.debug('cleaning up')

            terminate_task.cancel()
            # self.events.shutdown(immediate=True) py 3.12 only?
            
            # Cancel any pending timeouts
            for name, task in list(self.pending_timeouts.items()):
                task.cancel()
                self.log.debug(f"Cancelled pending timeout: {name}")
            
            self.log.info("Shutting down")
            self.mgr.shutdown()
            self.log_manager.close()
            self.nt.stopClient()

            self.pins.set_index_high(None) # deselect both tof sensors
