#!/usr/bin/env python3
'''Extract tof logs and output in wpilog format'''

# pylint: disable=invalid-name,trailing-whitespace,missing-module-docstring
# pylint: disable=missing-function-docstring,missing-class-docstring,no-member
# pylint: disable=consider-using-with,too-few-public-methods,unused-import
# pylint: disable=too-many-locals,line-too-long,too-many-positional-arguments
# pylint: disable=too-many-arguments

import argparse
import re
import os
import datetime as dt
import time
from pathlib import Path

import wpiutil
from wpiutil.log import DoubleLogEntry, StringLogEntry

def parse_wall_clock_time(time_str):
    """Convert wall clock time string (HH:MM:SS.sss) to seconds since midnight."""
    hours, minutes, seconds = time_str.split(':')
    total_seconds = int(hours) * 3600 + int(minutes) * 60 + float(seconds)
    return total_seconds

def calculate_mono_time(wall_time, latest_wall_time, latest_mono_time):
    """Calculate monotonic time based on wall clock time delta from reference."""
    if latest_wall_time is None or latest_mono_time is None:
        return None
    time_diff = wall_time - latest_wall_time
    return latest_mono_time + time_diff

def handle_mode_line(line, pattern, log_entry, mode_type, latest_times, verbose=False):
    """Handle mode update lines (tof mode or chute mode)."""
    match = re.search(pattern, line)
    if not match:
        return False, None
        
    mode_value = match.group(1)
    wall_time_str, wall_time = latest_times['current_wall_time_str'], latest_times['current_wall_time']
    
    # Calculate approximate monotonic time
    mono_time = calculate_mono_time(
        wall_time, 
        latest_times['latest_wall_time'], 
        latest_times['latest_mono_time']
    )
    
    if mono_time is not None:
        # Append with calculated timestamp
        log_entry.append(mode_value, int(mono_time * 1_000_000))
        if verbose:
            print(f"{wall_time_str}: Found {mode_type}: {mode_value} (mono time: {mono_time:.3f})")
    else:
        if verbose:
            print(f"{wall_time_str}: Skipping {mode_type} update (no reference timestamp): {mode_value}")
    
    # For tof mode, return whether we're in corner mode
    if mode_type == "tof mode":
        return True, (mode_value == "corner")
    return True, None

def parse_log_file(input_file, verbose=False):
    """Parse the log file and create a wpilog file."""
    # Generate output filename
    base_name = os.path.splitext(input_file)[0]
    output_file = f"{base_name}.wpilog"
    
    # Create DataLogWriter
    dlog = wpiutil.DataLogWriter(output_file)
    
    # Create log entries
    dist_entry = DoubleLogEntry(dlog, "/Pi/dist_mm")
    corner_entry = DoubleLogEntry(dlog, "/Pi/corner")
    chute_mode_entry = StringLogEntry(dlog, "/Pi/chute_mode")
    tof_mode_entry = StringLogEntry(dlog, "/Pi/tof_mode")
    
    # Read the log file
    with open(input_file, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    
    # Flag to track if we're in "corner" mode
    in_corner_mode = False
    
    # Dictionary to track the latest timestamps
    latest_times = {
        'latest_wall_time': None,
        'latest_mono_time': None,
        'current_wall_time': None,
        'current_wall_time_str': None
    }
    
    # Process each line
    for line in lines:
        # Extract wall clock time from the beginning of the line
        wall_time_match = re.match(r'(\d+:\d+:\d+\.\d+)', line)
        if not wall_time_match:
            continue
            
        wall_time_str = wall_time_match.group(1)
        wall_time = parse_wall_clock_time(wall_time_str)
        
        # Update current wall time
        latest_times['current_wall_time_str'] = wall_time_str
        latest_times['current_wall_time'] = wall_time
        
        # Check for chute mode updates
        handled, _ = handle_mode_line(
            line, 
            r'INFO:tof: chute: (.+)', 
            chute_mode_entry, 
            "chute mode", 
            latest_times,
            verbose
        )
        if handled:
            continue
        
        # Check for tof mode updates
        handled, tof_mode_result = handle_mode_line(
            line, 
            r'INFO:tof: tof mode: (.+)', 
            tof_mode_entry, 
            "tof mode", 
            latest_times,
            verbose
        )
        if handled:
            in_corner_mode = tof_mode_result
            continue
        
        # Check for distance readings to update our reference timestamps
        dist_match = re.search(r'INFO:tof: dist,(\d+\.\d+),\s*(\d+)', line)
        if dist_match:
            mono_time = float(dist_match.group(1))
            distance = float(dist_match.group(2))
            
            # Update reference timestamps
            latest_times['latest_wall_time'] = wall_time
            latest_times['latest_mono_time'] = mono_time
            
            # Only process data if in corner mode
            if in_corner_mode:
                dist_entry.append(distance, int(mono_time * 1_000_000))
                if verbose:
                    print(f"{wall_time_str}: Recorded distance: {distance} mm at {mono_time}")
            continue
        
        # Only process corner readings if in corner mode
        if in_corner_mode:
            # Check for corner readings (using the tof format)
            corner_match = re.search(r'INFO:tof: CORNER: (\d+\.\d+),(\d+\.\d+)', line)
            if corner_match:
                mono_time = float(corner_match.group(1))
                corner_value = float(corner_match.group(2))
                
                # Update reference timestamps
                latest_times['latest_wall_time'] = wall_time
                latest_times['latest_mono_time'] = mono_time
                
                corner_entry.append(corner_value, int(mono_time * 1_000_000))
                if verbose:
                    print(f"{wall_time_str}: Recorded corner: {corner_value} at {mono_time}")
                continue
    
    # Flush and close the log
    dlog.flush()
    
    print(f"Converted {input_file} to {output_file}")

def main():
    parser = argparse.ArgumentParser(description='Convert TOF log files to wpilog format')
    parser.add_argument('files', nargs='+', help='Input log file(s)')
    parser.add_argument('-v', '--verbose', action='store_true', help='Print verbose output')
    args = parser.parse_args()
    
    for input_file in args.files:
        print(f'{input_file!r}')
        if not Path(input_file).is_file():
            print(f"Warning: '{input_file}' does not exist or not a file. Skipping.")
            continue
            
        parse_log_file(input_file, args.verbose)

if __name__ == "__main__":
    main()
    
