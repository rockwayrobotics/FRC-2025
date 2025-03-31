import datetime
import logging
import time
from pathlib import Path

import ntcore
import wpiutil
from wpiutil.log import DoubleLogEntry, StringLogEntry, DoubleArrayLogEntry

from .names import Name

LOG_DIR = Path("logs")


class EntryType:
    DOUBLE = "double"
    STRING = "string"
    DOUBLE_ARRAY = "double_array"

class LogManager:
    """
    Manages DataLogBackgroundWriter for logging TOF sensor data with smart log cycling
    """
    def __init__(self, log_dir=Path("logs"), cycle_minutes=30):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(exist_ok=True, parents=True)
        self.log = logging.getLogger("log")
        self.writer = None
        
        # Time-based cycling configuration
        self.time_cycle_interval = cycle_minutes * 60  # Convert to seconds
        self.last_cycle_time = time.monotonic()
        
        # Initialize entries dict to track all created entries
        self.entries = {}
        
        # Match context for filename generation
        self.fms_attached = False
        self.match_type = None
        self.match_number = None
        self.replay_number = None
        self.event_name = None
        
        # Flag for waiting on match context
        self.awaiting_match_context = False
        self.awaiting_match_context_since = None
        self.awaiting_match_context_timeout = 120  # 2 minutes
        
        # Current filename without path
        self.current_filename = None
        
        # Define initial entries configuration - centralized definition
        self.entry_config = {
            Name.DIST_MM: {"type": EntryType.DOUBLE},
            Name.TS_DIST_MM: {"type": EntryType.DOUBLE_ARRAY},
            Name.CORNER: {"type": EntryType.DOUBLE},
            Name.CORNER_DIST_MM: {"type": EntryType.DOUBLE},
            Name.TOF_MODE: {"type": EntryType.STRING},
            Name.CHUTE_MODE: {"type": EntryType.STRING},
        }
        
        # Create initial log file
        self.start_new_log()
    
    def start_new_log(self):
        """Start a new log file with timestamp-based name"""
        timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        filename = f"tof-{timestamp}.wpilog"
        self.log.info(f"Starting new log file: {filename}")
        
        # If we have an existing writer, pause it before switching
        if self.writer:
            self.writer.pause()
        
        # Create new writer
        self.writer = wpiutil.DataLogBackgroundWriter(
            str(self.log_dir),  # Directory 
            filename,           # Filename
            period=0.5,         # Flush period in seconds
            extraHeader="TOF Sensor Data"
        )
        
        # Update tracking
        self.current_filename = filename
        self.last_cycle_time = time.monotonic()
        
        # Set flag to watch for match context
        self.awaiting_match_context = True
        self.awaiting_match_context_since = time.monotonic()
        
        # Re-create all entries with the new writer
        self._recreate_entries()
        
        return filename
    
    def _recreate_entries(self):
        """Recreate all log entries with the new writer"""
        # Clear current entries
        self.entries = {}
        
        # Create each entry from the configuration
        for name, config in self.entry_config.items():
            self._create_entry(
                name, 
                config["type"],
                metadata=config.get("metadata", None)
            )
    
    def _create_entry(self, name, entry_type, metadata=None):
        """Internal method to create a new log entry"""
        if entry_type == EntryType.DOUBLE:
            if metadata:
                entry = DoubleLogEntry(self.writer, name, metadata)
            else:
                entry = DoubleLogEntry(self.writer, name)
        elif entry_type == EntryType.STRING:
            if metadata:
                entry = StringLogEntry(self.writer, name, metadata)
            else:
                entry = StringLogEntry(self.writer, name)
        elif entry_type == EntryType.DOUBLE_ARRAY:
            if metadata:
                entry = DoubleArrayLogEntry(self.writer, name, metadata)
            else:
                entry = DoubleArrayLogEntry(self.writer, name)
        else:
            raise ValueError(f"Unsupported entry type: {entry_type}")
            
        # Store entry for future use
        self.entries[name] = entry
        
        return entry
    
    def update_match_context(self, event_name=None, match_type=None, match_number=None, replay_number=None):
        """Update match context data for filename generation"""
        updated = False
        
        if event_name is not None and event_name != self.event_name:
            self.event_name = event_name
            updated = True
            
        if match_type is not None and match_type != self.match_type:
            self.match_type = match_type
            updated = True
            
        if match_number is not None and match_number != self.match_number:
            self.match_number = match_number
            updated = True
            
        if replay_number is not None and replay_number != self.replay_number:
            self.replay_number = replay_number
            updated = True
            
        if updated and self.awaiting_match_context:
            self.check_rename_log()
            
        return updated
    
    def set_fms_attached(self, attached):
        """Update FMS attachment state and handle log file actions"""
        if attached == self.fms_attached:
            return False  # No change
            
        self.fms_attached = attached
        
        if attached:
            # FMS has just been attached
            if not self.awaiting_match_context:
                # We need to start a new log for this match
                self.log.info("FMS attached - starting new log file")
                self.start_new_log()
            else:
                # We're already in a new log and awaiting context
                # Let the match context update and checkRenameLog handle it
                self.log.info("FMS attached while awaiting match context")
        else:
            # FMS has been detached
            # No immediate action needed - next log will be created on:
            # - Time basis
            # - Next FMS attachment
            # - Rio reconnection
            self.log.info("FMS detached")
        
        return True
    
    def check_rename_log(self):
        """Check if we should rename the log based on match context"""
        if not self.awaiting_match_context:
            return False
            
        # Check if we have enough match information
        if self.fms_attached and self.match_type is not None and self.match_number is not None:
            # We have enough info to rename the file with match context
            match_type_str = {
                1: "p",  # Practice
                2: "q",  # Qualification
                3: "e",  # Playoff/Elimination
            }.get(self.match_type, str(self.match_type))
            
            event_name_str = self.event_name.lower() if self.event_name else "unknown"
            replay_suffix = f"-{self.replay_number}" if self.replay_number and self.replay_number > 1 else ""
            
            # Get current timestamp part from filename
            timestamp = self.current_filename.split("-", 2)[1]
            
            # Build new filename
            new_filename = f"tof-{timestamp}-{event_name_str}-{match_type_str}{self.match_number}{replay_suffix}.wpilog"
            
            if new_filename != self.current_filename:
                self.log.info(f"Renaming log file to: {new_filename}")
                self.writer.setFilename(new_filename)
                self.current_filename = new_filename
                
            # Clear the awaiting flag since we've handled it
            self.awaiting_match_context = False
            self.awaiting_match_context_since = None
            
            return True
            
        return False
    
    def check_cycle_timeout(self, current_tof_mode):
        """Check if awaiting match context has timed out or if time-based cycling is needed"""
        now = time.monotonic()
        
        # Check for match context timeout
        if self.awaiting_match_context and (now - self.awaiting_match_context_since) > self.awaiting_match_context_timeout:
            self.log.info("Match context wait timeout - no FMS attachment detected")
            self.awaiting_match_context = False
            self.awaiting_match_context_since = None
            
        # Check for time-based cycling - skip if in corner mode
        if current_tof_mode != "corner" and (now - self.last_cycle_time) >= self.time_cycle_interval:
            self.log.info(f"Time-based log cycling after {self.time_cycle_interval/60:.1f} minutes")
            self.start_new_log()
            return True
            
        return False
    
    def append_double(self, name, value, timestamp):
        """Append a double value to the specified entry"""
        if name not in self.entries:
            self.log.warning(f"Entry {name} does not exist")
            return False
            
        try:
            self.entries[name].append(value, int(timestamp * 1000000))
            return True
        except Exception as e:
            self.log.error(f"Error appending to {name}: {e}")
            return False
    
    def append_string(self, name, value, timestamp):
        """Append a string value to the specified entry"""
        if name not in self.entries:
            self.log.warning(f"Entry {name} does not exist")
            return False
            
        try:
            self.entries[name].append(value, int(timestamp * 1000000))
            return True
        except Exception as e:
            self.log.error(f"Error appending to {name}: {e}")
            return False
    
    def append_double_array(self, name, value, timestamp):
        """Append a double array value to the specified entry"""
        if name not in self.entries:
            self.log.warning(f"Entry {name} does not exist")
            return False
            
        try:
            self.entries[name].append(value, int(timestamp * 1000000))
            return True
        except Exception as e:
            self.log.error(f"Error appending to {name}: {e}")
            return False
    
    def close(self):
        """Close the log writer"""
        if self.writer:
            self.writer.stop()
            self.writer = None
