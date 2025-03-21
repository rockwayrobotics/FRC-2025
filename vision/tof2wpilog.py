#!/usr/bin/env python3
'''Extract tof logs and output in wpilog format'''

# pylint: disable=invalid-name,trailing-whitespace,missing-module-docstring
# pylint: disable=missing-function-docstring,missing-class-docstring,no-member
# pylint: disable=consider-using-with,too-few-public-methods,unused-import
# pylint: disable=too-many-locals,line-too-long,too-many-positional-arguments
# pylint: disable=too-many-arguments,broad-exception-caught,redundant-keyword-arg
# pylint: disable=too-many-nested-blocks

#!/usr/bin/env python3

import argparse
import re
import os
import sys
import time
from pathlib import Path

import wpiutil
from wpiutil.log import DoubleLogEntry, StringLogEntry

def parse_wall_clock_time(time_str):
    """Convert wall clock time string (HH:MM:SS.sss) to seconds since midnight."""
    try:
        hours, minutes, seconds = time_str.split(':')
        total_seconds = int(hours) * 3600 + int(minutes) * 60 + float(seconds)
        return total_seconds
    except ValueError as e:
        print(f"Error parsing time string '{time_str}': {e}")
        return 0

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
        try:
            # Append with calculated timestamp
            log_entry.append(mode_value, int(mono_time * 1_000_000))
            if verbose:
                print(f"{wall_time_str}: Found {mode_type}: {mode_value} (mono time: {mono_time:.3f})")
        except Exception as e:
            print(f"Error writing {mode_type} entry: {e}")
    else:
        if verbose:
            print(f"{wall_time_str}: Skipping {mode_type} update (no reference timestamp): {mode_value}")
    
    # For tof mode, return whether we're in corner mode
    if mode_type == "tof mode":
        return True, (mode_value == "corner")
    return True, None

def parse_log_file(input_file, args):
    """Parse the log file and create a wpilog file."""
    # Generate output filename
    base_name = os.path.splitext(input_file)[0]
    output_file = f"{base_name}.wpilog"

    outpath = Path(output_file)
    if outpath.exists():
        outpath.unlink()
    
    # Create DataLogBackgroundWriter, which handles flushing in a background thread
    # This should avoid the deadlock issues of DataLogWriter
    try:
        print(f"Creating log file: {output_file}")
        dlog = wpiutil.DataLogBackgroundWriter(str(outpath.parent), 
            str(outpath.name),
            period=0.5)
        print("Log file created successfully")
    except Exception as e:
        print(f"Error creating DataLogBackgroundWriter for {output_file}: {e}")
        return None
    
    # Create log entries
    try:
        dist_entry = DoubleLogEntry(dlog, "/Pi/dist_mm")
        corner_entry = DoubleLogEntry(dlog, "/Pi/corner")
        chute_mode_entry = StringLogEntry(dlog, "/Pi/chute_mode")
        tof_mode_entry = StringLogEntry(dlog, "/Pi/tof_mode")
    except Exception as e:
        print(f"Error creating log entries: {e}")
        return None
    
    # Read the log file
    try:
        print(f"Reading {input_file}...")
        with open(input_file, 'r', encoding='utf-8') as f:
            lines = f.readlines()
        print(f"Read {len(lines)} lines from {input_file}")
    except Exception as e:
        print(f"Error reading {input_file}: {e}")
        return None
    
    # Flag to track if we're in "corner" mode
    in_corner_mode = False
    
    # Dictionary to track the latest timestamps and values
    latest_times = {
        'latest_wall_time': None,
        'latest_mono_time': None,
        'current_wall_time': None,
        'current_wall_time_str': None
    }
    
    # Track the most recent distance reading
    most_recent_distance = 0
    
    # Track pending CD corner (waiting for TOF corner)
    pending_cd_corner = None
    
    # For progress reporting
    total_lines = len(lines)
    last_progress = 0
    start_time = time.time()
    
    # Process each line
    for i, line in enumerate(lines):
        # Report progress every 10%
        progress = (i * 100) // total_lines
        if progress >= last_progress + 10:
            elapsed = time.time() - start_time
            print(f"Processing {progress}% complete ({i}/{total_lines} lines), elapsed: {elapsed:.1f}s")
            last_progress = progress
            # Force flush to ensure output is visible
            sys.stdout.flush()
        
        try:
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
                args.verbose
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
                args.verbose
            )
            if handled:
                in_corner_mode = tof_mode_result
                # If we're exiting corner mode, clear any pending CD corner
                if not in_corner_mode:
                    pending_cd_corner = None
                continue
            
            # Check for distance readings to update our reference timestamps
            dist_match = re.search(r'INFO:tof: dist,(\d+\.\d+),\s*(\d+)', line)
            if dist_match:
                try:
                    mono_time = float(dist_match.group(1))
                    distance = float(dist_match.group(2))
                    
                    # Update reference timestamps
                    latest_times['latest_wall_time'] = wall_time
                    latest_times['latest_mono_time'] = mono_time
                    
                    # Update most recent distance
                    most_recent_distance = distance
                    
                    # Process data if in corner mode or if all distances are requested
                    if in_corner_mode or args.all_dists:
                        dist_entry.append(distance, int(mono_time * 1_000_000))
                        if args.verbose:
                            mode_str = "in corner mode" if in_corner_mode else "all dist option"
                            print(f"{wall_time_str}: Recorded distance: {distance} mm at {mono_time} ({mode_str})")
                except Exception as e:
                    print(f"Error processing distance line {i+1}: {line.strip()}")
                    print(f"Exception: {e}")
                continue
            
            # Check for cd: CORNER line (first detection)
            cd_corner_match = re.search(r'INFO:cd: CORNER: (\d+\.\d+)', line)
            if cd_corner_match and in_corner_mode:
                try:
                    # Store CD corner info, but don't output yet - wait for TOF corner
                    cd_mono_time = float(cd_corner_match.group(1))
                    pending_cd_corner = (cd_mono_time, most_recent_distance)
                    
                    if args.verbose:
                        print(f"{wall_time_str}: Found CD corner at {cd_mono_time}, waiting for TOF corner")
                    
                    # Update reference timestamps
                    latest_times['latest_wall_time'] = wall_time
                    latest_times['latest_mono_time'] = cd_mono_time
                except Exception as e:
                    print(f"Error processing CD corner line {i+1}: {line.strip()}")
                    print(f"Exception: {e}")
                continue
            
            # Check for tof: CORNER line (processing time)
            tof_corner_match = re.search(r'INFO:tof: CORNER: (\d+\.\d+)', line)
            if tof_corner_match and in_corner_mode:
                try:
                    tof_mono_time = float(tof_corner_match.group(1))
                    
                    # Only process if we have a pending CD corner
                    if pending_cd_corner:
                        cd_mono_time, corner_value = pending_cd_corner
                        
                        # Output CD corner point
                        corner_entry.append(corner_value, int(cd_mono_time * 1_000_000))
                        
                        # Output TOF corner point (reset to zero)
                        corner_entry.append(0.0, int(tof_mono_time * 1_000_000))
                        
                        if args.verbose:
                            print(f"{wall_time_str}: Processed corner pair - CD at {cd_mono_time} with value {corner_value}, TOF at {tof_mono_time}")
                        
                        # Clear pending corner
                        pending_cd_corner = None
                    else:
                        if args.verbose:
                            print(f"{wall_time_str}: Found TOF corner at {tof_mono_time} but no matching CD corner, ignoring")
                    
                    # Update reference timestamps
                    latest_times['latest_wall_time'] = wall_time
                    latest_times['latest_mono_time'] = tof_mono_time
                except Exception as e:
                    print(f"Error processing TOF corner line {i+1}: {line.strip()}")
                    print(f"Exception: {e}")
                continue
                
        except Exception as e:
            print(f"Error processing line {i+1}: {line.strip()}")
            print(f"Exception: {e}")
    
    print(f"Converted {input_file} to {output_file}")

def main():
    parser = argparse.ArgumentParser(description='Convert TOF log files to wpilog format')
    parser.add_argument('-a', '--all-dists', action='store_true', 
                        help='Include all distance readings, not just those in corner mode')
    parser.add_argument('files', nargs='+', help='Input log file(s)')
    parser.add_argument('-v', '--verbose', action='store_true', help='Print verbose output')
    args = parser.parse_args()
    
    for input_file in args.files:
        if not Path(input_file).is_file():
            print(f"Warning: '{input_file}' does not exist or not a file. Skipping.")
            continue
            
        parse_log_file(input_file, args)

if __name__ == "__main__":
    main()
    
