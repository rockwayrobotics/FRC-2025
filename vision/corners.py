#!/usr/bin/env python3
"""
Extract corner detection blocks from wpilog files and output structured data in TOML format.

This utility extracts sections where tof_mode is "corner", identifies corner detections,
and outputs them in a structured TOML format for analysis and comparison with
different corner detection algorithms.
"""

import argparse
from datetime import datetime
import importlib.util
import logging
import math
from pathlib import Path
import sys

import numpy as np
import toml
from wpiutil.log import DataLogReader

# Dynamically import the CornerDetector class
def import_corner_detector(name=None):
    """Import the CornerDetector class from the tof package."""
    # Map version names to module paths
    module_paths = {
        'v1': "tof.corner_detector",
        'v2': "tof.corner_detector2",
        'v3': "tof.cd3",
    }
    
    module_name = module_paths.get(name)
    
    if not module_name:
        raise ValueError(f"Unknown corner detector version: {name}")
    
    try:
        # Import it directly from the package
        module = importlib.import_module(module_name)
        return module.CornerDetector
    except ImportError as e:
        # Log the error for debugging
        import logging
        logging.warning(f"Failed to import CornerDetector from {module_name}: {e}")
        raise


class CornerExtractor:
    def __init__(self, speed=450.0, debug=False, name=None):
        """Initialize the corner extractor.
        
        Args:
            speed: Robot speed in mm/s for angle calculations
            debug: Enable debug logging
            corner_detector_path: Optional path to corner_detector.py
        """
        self.speed = speed
        self.log = logging.getLogger("corners")
        self.log_level = logging.DEBUG if debug else logging.INFO
        
        # Configure logging
        logging.basicConfig(
            level=self.log_level,
            format="%(levelname)5s: %(message)s",
            stream=sys.stdout,
        )
        
        # Import the CornerDetector class
        try:
            self.cd_class = import_corner_detector(name)
            self.log.info("Successfully imported CornerDetector %s", name)
        except Exception as e:
            self.log.warning(f"Failed to import CornerDetector: {e}")
            self.cd_class = None
    
    def extract_blocks(self, wpilog_path):
        """Extract corner mode blocks from a wpilog file.
        
        Args:
            wpilog_path: Path to the wpilog file
            
        Returns:
            List of blocks containing corner mode data
        """
        self.log.info(f"Extracting corner mode blocks from {wpilog_path}")
        
        try:
            # Create the log reader
            reader = DataLogReader(str(wpilog_path))
            
            # Data structures to track entry metadata
            entries = {}  # Map of entry ID to metadata
            entry_name_to_id = {}  # Map of entry name to ID
            
            # Data structures for corner mode blocks
            blocks = []
            current_block = {}
            in_corner_mode = False
            
            # Process records in a single pass
            for record in reader:
                if record.isStart():
                    # Handle start records (build metadata)
                    try:
                        data = record.getStartData()
                        self.log.debug(f"Found entry: {data.name} (ID: {data.entry}, Type: {data.type})")
                        
                        entries[data.entry] = data
                        entry_name_to_id[data.name] = data.entry
                    except TypeError as e:
                        self.log.warning(f"Invalid start record: {e}")
                    
                elif not record.isFinish() and not record.isSetMetadata():
                    # Handle data records
                    timestamp_us = record.getTimestamp()
                    timestamp = timestamp_us / 1_000_000  # Convert to seconds
                    entry_id = record.getEntry()
                    
                    # Skip if we don't have metadata for this entry
                    if entry_id not in entries:
                        continue
                    
                    entry_metadata = entries[entry_id]
                    entry_name = entry_metadata.name
                    
                    # Process based on entry type
                    try:
                        if entry_name == "/Pi/tof_mode":
                            mode = record.getString()
                            
                            if mode == "corner" and not in_corner_mode:
                                # Start a new corner mode block
                                in_corner_mode = True
                                current_block = {
                                    "begin": timestamp,
                                    "end": None,
                                    "distances": [],
                                    "corners": []
                                }
                                num = len(blocks) + 1
                                self.log.debug(f"Starting corner block {num} at {timestamp:.3f}")
                            
                            elif mode != "corner" and in_corner_mode:
                                # End the current corner mode block
                                in_corner_mode = False
                                if current_block:
                                    current_block["end"] = timestamp
                                    self.log.debug(f"Ending corner block at {timestamp:.3f}")
                                    
                                    # Only add blocks with distance readings
                                    if current_block["distances"]:
                                        blocks.append(current_block)
                                        self.log.debug(f"Added block: {current_block['begin']:.3f}-{current_block['end']:.3f} "
                                                    f"with {len(current_block['distances'])} distance readings and "
                                                    f"{len(current_block['corners'])} corners")
                                    else:
                                        self.log.warning(f"Skipping empty block: {current_block['begin']:.3f}-{current_block['end']:.3f}")
                                    
                                    current_block = None
                        
                        elif entry_name == "/Pi/dist_mm" and in_corner_mode and current_block:
                            # Add distance reading to the current block
                            distance = record.getDouble()
                            current_block["distances"].append({
                                "time": timestamp,
                                "dist": distance
                            })
                        
                        elif entry_name == "/Pi/corner" and in_corner_mode and current_block:
                            # Process corner detection
                            value = record.getDouble()
                            if value > 0:  # Non-zero value indicates a corner detection
                                corner_info = {
                                    "c_time": timestamp,
                                    "c_dist": value,
                                    "c_angle": None  # Will calculate later
                                }
                                current_block["corners"].append(corner_info)
                                self.log.debug(f"Found corner at {timestamp:.3f} with distance {value:.1f}")
                        
                        elif entry_name == "/Pi/corner_ts" and in_corner_mode and current_block:
                            # Update corner timestamp if we have corners
                            if current_block["corners"]:
                                corner_ts_value = record.getFloat()
                                # Find the most recently added corner and update its time
                                for corner in reversed(current_block["corners"]):
                                    if abs(corner["c_time"] - timestamp) < 1.0:  # Within 1 second
                                        corner["c_time"] = corner_ts_value
                                        self.log.debug(f"Updated corner time to {corner_ts_value:.3f}")
                                        break
                        
                        elif entry_name == "/Pi/corner_dist_mm" and in_corner_mode and current_block:
                            # Update corner distance if we have corners
                            if current_block["corners"]:
                                corner_dist_value = record.getFloat()
                                # Find the most recently added corner and update its distance
                                for corner in reversed(current_block["corners"]):
                                    if abs(corner["c_time"] - timestamp) < 1.0:  # Within 1 second
                                        corner["c_dist"] = corner_dist_value
                                        self.log.debug(f"Updated corner distance to {corner_dist_value:.1f}")
                                        break
                    
                    except TypeError as e:
                        self.log.exception(f"Error processing entry {entry_name}: {e}")
            
            # Handle case where log ends during corner mode
            if in_corner_mode and current_block:
                current_block["end"] = timestamp
                self.log.warning(f"Log ended during corner mode. Using {timestamp:.3f} as end time.")
                
                if current_block["distances"]:
                    blocks.append(current_block)
                    self.log.info(f"Added final block: {current_block['begin']:.3f}-{current_block['end']:.3f}")
            
            # Keep only the first corner in each block (as specified in requirements)
            for block in blocks:
                if len(block["corners"]) > 1:
                    self.log.debug(f"Block {block['begin']:.3f}-{block['end']:.3f} has "
                                 f"{len(block['corners'])} corners, keeping only the first one")
                    block["corners"] = [block["corners"][0]]
            
            self.log.info(f"Extracted {len(blocks)} corner mode blocks")
            return blocks
        
        except Exception as e:
            self.log.exception(f"Error extracting corner blocks: {e}")
            return []
    
    def run_corner_detector(self, distances, speed):
        """Run the CornerDetector algorithm on a sequence of distance readings.
        
        Args:
            distances: List of (time, distance) dictionaries
            speed: Speed in mm/s
            
        Returns:
            Dictionary with corner information or None if no corner detected
        """
        if self.cd_class is None:
            self.log.warning("CornerDetector is not available, skipping corner detection")
            return None
        
        try:
            # Create a new detector with appropriate settings
            # The params are from tof/main.py - slope_threshold=400
            detector = self.cd_class(400)
            
            self.log.debug(f"Running CornerDetector on {len(distances)} distance readings with speed={speed}")
            
            # Sort distances by time if they aren't already
            sorted_distances = sorted(distances, key=lambda x: x["time"])
            
            # Feed each distance reading to the detector
            for point in sorted_distances:
                detector.add_record(point["time"], point["dist"], speed / 1000)  # Convert to m/s
                
                # Check if a corner was found
                if detector.found_corner():
                    # Get the corner information
                    corner = {
                        "c_time": detector.corner_timestamp,
                        "c_dist": detector.corner_dist,
                        "c_angle": math.degrees(detector.corner_angle) if detector.corner_angle is not None else None
                    }
                    
                    self.log.info(f"CornerDetector found corner at {corner['c_time']:.3f} with distance {corner['c_dist']:.1f}")
                    
                    # Reset the detector and return the corner
                    detector.reset()
                    return corner
            
            self.log.info("CornerDetector did not find any corners")
            return None
            
        except Exception as e:
            self.log.exception(f"Error running CornerDetector: {e}")
            return None
    
    def calculate_angle(self, distances, corner_time, window_size=10):
        """Calculate the angle at a corner based on distance measurements.
        
        Args:
            distances: List of (time, distance) tuples
            corner_time: Time of the corner
            window_size: Size of window to use for slope calculation
            
        Returns:
            Angle in degrees or None if cannot be calculated
        """
        # This is a simplified implementation - in practice, you'd use the
        # regression approach from the CornerDetector class
        
        # Find distances near the corner time
        before_corner = []
        after_corner = []
        
        for d in distances:
            time = d["time"]
            if time < corner_time:
                before_corner.append(d)
            elif time > corner_time:
                after_corner.append(d)
        
        # Get the closest points before and after
        before_corner = sorted(before_corner, key=lambda x: corner_time - x["time"])[:window_size]
        after_corner = sorted(after_corner, key=lambda x: x["time"] - corner_time)[:window_size]
        
        if len(before_corner) < 3 or len(after_corner) < 3:
            self.log.warning(f"Not enough points to calculate angle at {corner_time:.3f}")
            return None
        
        # Calculate slopes using simple linear regression
        # Before corner
        before_times = [d["time"] for d in before_corner]
        before_distances = [d["dist"] for d in before_corner]
        before_mean_time = sum(before_times) / len(before_times)
        before_mean_dist = sum(before_distances) / len(before_distances)
        before_numerator = sum((t - before_mean_time) * (d - before_mean_dist) for t, d in zip(before_times, before_distances))
        before_denominator = sum((t - before_mean_time) ** 2 for t in before_times)
        before_slope = before_numerator / before_denominator if before_denominator != 0 else 0
        
        # After corner
        after_times = [d["time"] for d in after_corner]
        after_distances = [d["dist"] for d in after_corner]
        after_mean_time = sum(after_times) / len(after_times)
        after_mean_dist = sum(after_distances) / len(after_distances)
        after_numerator = sum((t - after_mean_time) * (d - after_mean_dist) for t, d in zip(after_times, after_distances))
        after_denominator = sum((t - after_mean_time) ** 2 for t in after_times)
        after_slope = after_numerator / after_denominator if after_denominator != 0 else 0
        
        # Use the second (after) slope for angle calculation, as in the original code
        angle_rad = math.atan2(after_slope, self.speed)
        angle_deg = math.degrees(angle_rad)
        
        self.log.debug(f"Calculated angle at {corner_time:.3f}: {angle_deg:.2f}° (before_slope={before_slope:.1f}, after_slope={after_slope:.1f})")
        
        return angle_deg
    
    def generate_toml(self, blocks, toml_path, force=False):
        """Generate a TOML file with the corner data.
        
        Args:
            blocks: List of corner mode blocks
            toml_path: Path to the output TOML file
            use_corner_detector: Whether to use the CornerDetector algorithm
        """
        if Path(toml_path).exists() and not force:
            raise FileExistsError(f'file "{toml_path}" exists! Use --force to overwrite.')
            
        self.log.info(f"Generating TOML file: {toml_path}")
        
        toml_data = {
            "metadata": {
                "generated_at": datetime.now().isoformat(),
                "speed_mm_s": self.speed,
                "block_count": len(blocks),
            },
            "blocks": []
        }
        
        for i, block in enumerate(blocks):
            self.log.debug(f"Processing block {i+1}/{len(blocks)}")
            
            # Create the basic block structure
            toml_block = {
                "id": i + 1,
                "begin": block["begin"],
                "end": block["end"],
                "distances": len(block["distances"]),
                "tags": "",
            }
            
            # Include the original corner if available
            if block["corners"]:
                first_corner = block["corners"][0]
                
                # Calculate angle if not already set
                if first_corner["c_angle"] is None:
                    first_corner["c_angle"] = self.calculate_angle(
                        block["distances"], 
                        first_corner["c_time"]
                    )
                
                # Use original corner as expected corner initially
                toml_block.update({
                    "c_time": first_corner["c_time"],
                    "c_dist": first_corner["c_dist"],
                    "c_angle": first_corner["c_angle"]
                })
            
            # Run the corner detector if no corner time found in original output
            if self.cd_class is not None:
                detected_corner = self.run_corner_detector(block["distances"], self.speed)
                toml_block.update(detected_corner or {})
            
            toml_data["blocks"].append(toml_block)
        
        # Write the TOML file
        with open(toml_path, 'w', encoding='utf-8') as f:
            toml.dump(toml_data, f, encoder=toml.TomlNumpyEncoder())
        
        self.log.info(f"Generated TOML file with {len(toml_data['blocks'])} blocks")
        return toml_data
    
    def compare_corners(self, blocks, toml_path):
        """Compare detected corners with expected corners from TOML file.
        
        Args:
            detected_blocks: List of detected corner blocks
            expected_toml_path: Path to the TOML file containing expected corners
        """
        self.log.info(f"Comparing corners with expected data from {toml_path}")
        
        try:
            # Load expected data
            with open(toml_path, 'r', encoding='utf-8') as f:
                expected_data = toml.load(f)
            
            expected_blocks = expected_data.get("blocks", [])
            detected_blocks = []
            
            if len(blocks) != len(expected_blocks):
                self.log.warning(f"Number of blocks differs: input={len(blocks)}, toml={len(expected_blocks)}")
            
            stats = {
                "matched": 0,
                "missed": 0,
                "false": 0,

                "errors": [], # errors (diff) of results where comparison was possible
            }

            # Compare each block
            for i, (input, expected) in enumerate(zip(blocks, expected_blocks)):
                self.log.info(f"Block {i+1}:")
                
                # Compare timing
                delta = abs(input["begin"] - expected["begin"])
                if delta > 0.001:
                    self.log.warning(f"  Start time: {input['begin']:.3f} (expected: {expected['begin']:.3f}, diff: {delta:.3f}s)")
                
                delta = abs(input["end"] - expected["end"])
                if delta > 0.001:
                    self.log.warning(f"  End time: {input['end']:.3f} (expected: {expected['end']:.3f}, diff: {delta:.3f}s)")
                
                if len(input['distances']) != expected['distances']:
                    self.log.warning(f"  Distance count: {len(input['distances'])} (expected: {expected['distances']})")

                expected_corner = {k: expected[k] for k in "c_time c_dist c_angle".split() if k in expected}

                bad_tags = set(expected.get('tags', '').split(',')) & {'missed','short-intro'}
                if bad_tags:
                    text = ' + '.join(bad_tags)
                    self.log.warning(f'block tagged {text}: cannot do corner detection')
                    continue
                
                # Compare corners
                detected = self.run_corner_detector(input["distances"], self.speed)
                if detected and expected_corner:
                    detected_blocks.append(detected)
                    stats['matched'] += 1
                    
                    delta = detected["c_time"] - expected_corner["c_time"]
                    stats['errors'].append(delta)
                    
                    dist_diff = abs(detected["c_dist"] - expected_corner["c_dist"])
                    
                    self.log.info(f"  Corner Time: {detected['c_time']:.3f}s (expected: {expected_corner['c_time']:.3f}, diff: {delta:.3f})")
                    self.log.info(f"  Corner Dist: {detected['c_dist']:.1f}mm (expected: {expected_corner['c_dist']:.0f}, diff: {dist_diff:.0f})")
                    
                    detected_angle = detected.get("c_angle")
                    expected_angle = expected_corner.get("c_angle")
                    
                    if detected_angle is not None and expected_angle is not None:
                        angle_diff = abs(detected_angle - expected_angle)
                        self.log.debug(f"    Angle: {detected_angle:.1f}° (expected: {expected_angle:.1f}°, diff: {angle_diff:.1f}°)")
                    elif detected_angle is not None:
                        self.log.debug(f"    Angle: {detected_angle:.1f}° (no expected angle)")
                    elif expected_angle is not None:
                        self.log.debug(f"    Angle: Not calculated (expected: {expected_angle:.1f}°)")
                
                elif expected_corner:
                    stats['missed'] += 1
                    self.log.warning(f"  No corner detected (expected at {expected['c_time']:.3f})")
                
                elif detected:
                    stats['false'] += 1
                    self.log.warning(f"  Corner detected at {detected['corners'][0]['c_time']:.3f} (not expected)")
                
                else:
                    self.log.info("  No corners (as expected)")
            
            # Print summary
            self.log.info("Summary:")
            self.log.info(f"  Total blocks: {len(detected_blocks)}")
            self.log.info(f"  Matching corners: {stats['matched']}")
            self.log.info(f"  Missed corners: {stats['missed']}")
            self.log.info(f"  False positives: {stats['false']}")
            errors = np.array(stats['errors'])
            self.log.info(f"  c_dist error: mean={errors.mean():.3f} std={errors.std():.3f}")
        
        except Exception as e:
            self.log.exception(f"Error comparing corners: {e}")


def main():
    parser = argparse.ArgumentParser(description="Extract corner mode blocks from wpilog files")
    parser.add_argument("wpilog", help="Path to the wpilog file")
    parser.add_argument("-o", "--output",
        help="Path to the output TOML file (default: same as wpilog with .toml extension)")
    parser.add_argument("-g", "--generate", action="store_true",
        help="Generate a TOML file with expected corner data")
    parser.add_argument("-d", "--debug", action="store_true",
        help="Enable debug logging")
    parser.add_argument("--force", action="store_true", help="Force overwrite of output files")
    parser.add_argument("--speed", type=float, default=450.0,
        help="Assumed speed in mm/s for angle calculation (default: 450.0)")
    parser.add_argument("--cd", default="v1",
        help="CornerDetector algorithm to use (default '%(default)s')")
    args = parser.parse_args()
    
    # Check that the wpilog file exists
    wpilog_path = Path(args.wpilog)
    if not wpilog_path.exists():
        logging.error(f"Wpilog file not found: {wpilog_path}")
        sys.exit(1)
    
    # Determine the output TOML path
    toml_path = Path(args.output) if args.output else wpilog_path.with_suffix(".toml")
    
    # Check that the TOML file exists if not generating
    if not args.generate and not toml_path.exists():
        logging.error(f"TOML file not found: {toml_path}. Use --generate to create it.")
        sys.exit(1)
    
    # Create the corner extractor
    extractor = CornerExtractor(args.speed, args.debug, args.cd)
    
    # Extract blocks from the wpilog file
    blocks = extractor.extract_blocks(wpilog_path)
    
    if not blocks:
        logging.error("No corner mode blocks found in the wpilog file")
        sys.exit(1)
    
    if args.generate:
        # Generate the TOML file
        extractor.generate_toml(blocks, toml_path, force=args.force)
    else:
        # Compare against existing TOML file
        extractor.compare_corners(blocks, toml_path)

if __name__ == "__main__":
    main()
