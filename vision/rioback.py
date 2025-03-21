#!/usr/bin/env python3
"""
Asyncio script to backup files from a RoboRio to a Raspberry Pi using SSH.
Creates timestamped backup directories and tracks file changes to avoid unnecessary transfers.
"""

import asyncio
import asyncssh
import argparse
import logging
import os
import sys
import time
from datetime import datetime
from pathlib import Path
import json
import traceback

class RioBackup:
    """Class to handle backing up files from a RoboRio device."""
    
    def __init__(self, rio_address, username, backup_dir, max_retries=3, retry_delay=5):
        """Initialize the backup handler."""
        self.rio_address = rio_address
        self.username = username
        self.base_backup_dir = Path(backup_dir)
        self.current_backup_dir = None
        self.conn = None
        self.sftp = None
        self.max_retries = max_retries
        self.retry_delay = retry_delay
        
        # Set up logging
        self.logger = logging.getLogger("RioBackup")
        
        # File tracking
        self.tracking_dir = self.base_backup_dir / ".tracking"
        self.file_index_path = self.tracking_dir / "file_index.json"
        self.file_index = self._load_file_index()
        
        # Statistics
        self.stats = {
            "start_time": None,
            "end_time": None,
            "total_files": 0,
            "successful_files": 0,
            "failed_files": 0,
            "new_files": 0,
            "changed_files": 0,
            "unchanged_files": 0,
            "total_bytes": 0
        }
    
    def _load_file_index(self):
        """Load the file index from disk."""
        if not self.file_index_path.exists():
            return {}
        
        try:
            with open(self.file_index_path, 'r') as f:
                return json.load(f)
        except Exception as e:
            self.logger.error(f"Error loading file index: {e}")
            return {}
    
    def _save_file_index(self):
        """Save the file index to disk."""
        try:
            # Ensure parent directory exists
            self.tracking_dir.mkdir(parents=True, exist_ok=True)
            
            with open(self.file_index_path, 'w') as f:
                json.dump(self.file_index, f, indent=2)
        except Exception as e:
            self.logger.error(f"Error saving file index: {e}")
    
    async def connect_with_retry(self):
        """Connect to the RoboRio with retry logic."""
        retries = 0
        while retries <= self.max_retries:
            try:
                self.logger.info(f"Connecting to {self.username}@{self.rio_address}... (attempt {retries+1}/{self.max_retries+1})")
                
                # Connect without password (using key authentication)
                self.conn = await asyncssh.connect(
                    self.rio_address,
                    username=self.username,
                    known_hosts=None
                )
                self.sftp = await self.conn.start_sftp_client()
                self.logger.info("Connection established")
                return True
                
            except (OSError, asyncssh.Error) as e:
                retries += 1
                self.logger.warning(f"Connection failed: {e}")
                
                if retries <= self.max_retries:
                    self.logger.info(f"Retrying in {self.retry_delay} seconds...")
                    await asyncio.sleep(self.retry_delay)
                else:
                    self.logger.error(f"Failed to connect after {self.max_retries+1} attempts")
                    return False
    
    def create_backup_dir(self):
        """Create a timestamped backup directory."""
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        backup_dir = self.base_backup_dir / f"rio-{timestamp}"
        backup_dir.mkdir(parents=True, exist_ok=True)
        self.current_backup_dir = backup_dir
        self.logger.info(f"Created backup directory: {backup_dir}")
        return backup_dir
    
    async def find_last_backup_dir(self):
        """Find the most recent backup directory."""
        if not self.base_backup_dir.exists():
            return None
        
        directories = [d for d in self.base_backup_dir.iterdir() 
                      if d.is_dir() and d.name.startswith("rio-")]
        
        if not directories:
            return None
        
        # Sort by directory name (which includes timestamp)
        directories.sort(reverse=True)
        self.logger.debug(f"Last backup directory: {directories[0]}")
        return directories[0]
    
    async def get_file_info(self, remote_path):
        """Get information about a remote file."""
        try:
            attrs = await self.sftp.stat(remote_path)
            return {"size": attrs.size, "mtime": attrs.mtime}
        except Exception as e:
            self.logger.debug(f"Error getting file info for {remote_path}: {e}")
            return None
    
    async def calculate_file_signature(self, remote_path):
        """Calculate a signature for a remote file to detect changes."""
        info = await self.get_file_info(remote_path)
        if not info:
            return None
        
        # Use size and mtime as a simple signature
        # This isn't a cryptographic hash, but helps identify changes
        return f"{info['size']}_{info['mtime']}"
    
    async def copy_file_with_resume(self, remote_path, local_path, backup_index_key=None):
        """Copy a file from RoboRio to local backup directory with resume capability."""
        try:
            local_path.parent.mkdir(parents=True, exist_ok=True)
            
            # Get remote file info
            remote_info = await self.get_file_info(remote_path)
            if not remote_info:
                self.logger.error(f"Cannot get remote file info for {remote_path}")
                self.stats["failed_files"] += 1
                return False
            
            remote_size = remote_info["size"]
            self.logger.info(f"Copying {remote_path} ({remote_size} bytes) to {local_path}")
            
            # Check if local file exists and is partially downloaded
            local_size = 0
            if local_path.exists():
                local_size = local_path.stat().st_size
                
                if local_size == remote_size:
                    self.logger.info(f"File {local_path} already exists with correct size, skipping")
                    
                    # Update file index if needed
                    if backup_index_key:
                        signature = await self.calculate_file_signature(remote_path)
                        if signature:
                            self.file_index[backup_index_key] = {
                                "path": str(local_path),
                                "signature": signature,
                                "timestamp": datetime.now().isoformat()
                            }
                            self._save_file_index()
                    
                    self.stats["unchanged_files"] += 1
                    return True
                elif local_size > remote_size:
                    self.logger.warning(f"Local file {local_path} is larger than remote file, overwriting")
                    local_size = 0
            
            # If resuming, open file in append mode, otherwise write mode
            mode = "ab" if local_size > 0 else "wb"
            
            # Create a connection for file transfer
            async with self.sftp.open(remote_path, "rb") as remote_file:
                # Seek to position if resuming
                if local_size > 0:
                    self.logger.info(f"Resuming download from byte {local_size}")
                    await remote_file.seek(local_size)
                
                with open(local_path, mode) as local_file:
                    # Buffer size - 64KB is a good balance
                    buffer_size = 64 * 1024
                    bytes_transferred = local_size
                    
                    # Read and write in chunks
                    while True:
                        data = await remote_file.read(buffer_size)
                        if not data:
                            break
                        
                        local_file.write(data)
                        bytes_transferred += len(data)
                        
                        # Show progress for large files
                        if remote_size > 1024 * 1024:  # 1MB
                            percent = bytes_transferred / remote_size * 100
                            self.logger.debug(f"Transfer progress: {percent:.1f}% ({bytes_transferred}/{remote_size} bytes)")
            
            # Verify file size after transfer
            final_size = local_path.stat().st_size
            if final_size != remote_size:
                self.logger.error(f"Size mismatch after transfer: {final_size} != {remote_size} for {local_path}")
                self.stats["failed_files"] += 1
                return False
            
            # Update stats
            self.stats["successful_files"] += 1
            self.stats["total_bytes"] += final_size
            
            # Update file index if a key was provided
            if backup_index_key:
                signature = await self.calculate_file_signature(remote_path)
                if signature:
                    self.file_index[backup_index_key] = {
                        "path": str(local_path),
                        "signature": signature,
                        "timestamp": datetime.now().isoformat()
                    }
                    self._save_file_index()
            
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to copy {remote_path}: {e}")
            self.logger.debug(traceback.format_exc())
            self.stats["failed_files"] += 1
            return False
    
    async def list_wpilog_files(self, remote_dir="/U/logs"):
        """List all .wpilog files in the specified remote directory."""
        try:
            files = await self.sftp.listdir(remote_dir)
            wpilog_files = [f for f in files if f.endswith(".wpilog")]
            self.logger.debug(f"Found {len(wpilog_files)} .wpilog files")
            return wpilog_files
        except Exception as e:
            self.logger.error(f"Error listing wpilog files: {e}")
            return []
    
    async def is_file_changed(self, remote_path, index_key):
        """Check if a file has changed since the last backup."""
        # If the file isn't in our index, it's new
        if index_key not in self.file_index:
            self.stats["new_files"] += 1
            return True
        
        # Calculate current file signature
        current_signature = await self.calculate_file_signature(remote_path)
        if not current_signature:
            # If we can't get a signature, assume it changed to be safe
            self.stats["changed_files"] += 1
            return True
        
        # Compare with stored signature
        if current_signature != self.file_index[index_key].get("signature"):
            self.stats["changed_files"] += 1
            return True
        
        self.stats["unchanged_files"] += 1
        return False
    
    async def find_files_to_backup(self, wpilog_files):
        """Determine which .wpilog files need to be backed up."""
        files_to_backup = []
        
        for filename in wpilog_files:
            remote_path = f"/U/logs/{filename}"
            index_key = f"logs/{filename}"
            
            if await self.is_file_changed(remote_path, index_key):
                files_to_backup.append(filename)
                self.logger.debug(f"Will backup {filename} (new or changed)")
            else:
                self.logger.debug(f"Skipping {filename} (no changes)")
        
        return files_to_backup
    
    async def backup_wpilog_files(self):
        """Backup all new or modified .wpilog files."""
        # List all wpilog files on the RoboRio
        wpilog_files = await self.list_wpilog_files()
        
        # Determine which files need to be backed up
        files_to_backup = await self.find_files_to_backup(wpilog_files)
        
        # Update total files count
        self.stats["total_files"] += len(files_to_backup)
        
        # Create logs directory in the backup folder
        logs_dir = self.current_backup_dir / "logs"
        logs_dir.mkdir(exist_ok=True)
        
        # Copy each file that needs backing up
        for filename in files_to_backup:
            remote_path = f"/U/logs/{filename}"
            local_path = logs_dir / filename
            index_key = f"logs/{filename}"
            
            await self.copy_file_with_resume(remote_path, local_path, index_key)
        
        self.logger.info(f"Backed up {self.stats['successful_files']}/{len(files_to_backup)} wpilog files")
        return self.stats['successful_files']
    
    async def backup_regular_files(self):
        """Backup the regular files (networktables.json and FRC_UserProgram.log)."""
        files_to_backup = [
            "/home/lvuser/networktables.json",
            "/home/lvuser/FRC_UserProgram.log"
        ]
        
        # Update total files count
        self.stats["total_files"] += len(files_to_backup)
        
        for remote_path in files_to_backup:
            filename = os.path.basename(remote_path)
            local_path = self.current_backup_dir / filename
            index_key = filename
            
            # For regular files, always check if they've changed
            if await self.is_file_changed(remote_path, index_key):
                await self.copy_file_with_resume(remote_path, local_path, index_key)
            else:
                # Even if unchanged, create a copy in the current backup
                last_backup_dir = await self.find_last_backup_dir()
                if last_backup_dir:
                    src_path = last_backup_dir / filename
                    if src_path.exists():
                        try:
                            import shutil
                            shutil.copy2(src_path, local_path)
                            self.logger.debug(f"Copied unchanged file {filename} from previous backup")
                            self.stats["successful_files"] += 1
                        except Exception as e:
                            self.logger.error(f"Error copying from previous backup: {e}")
                            # Try to download directly
                            await self.copy_file_with_resume(remote_path, local_path, index_key)
        
        self.logger.info(f"Backed up regular files")
    
    def print_backup_summary(self):
        """Print a summary of the backup operation."""
        if not self.stats["start_time"] or not self.stats["end_time"]:
            return
        
        duration = self.stats["end_time"] - self.stats["start_time"]
        duration_seconds = duration.total_seconds()
        
        self.logger.info("=" * 50)
        self.logger.info("Backup Summary")
        self.logger.info("=" * 50)
        self.logger.info(f"Backup directory: {self.current_backup_dir}")
        self.logger.info(f"Start time: {self.stats['start_time'].strftime('%Y-%m-%d %H:%M:%S')}")
        self.logger.info(f"End time: {self.stats['end_time'].strftime('%Y-%m-%d %H:%M:%S')}")
        self.logger.info(f"Duration: {duration_seconds:.2f} seconds")
        self.logger.info(f"Files processed: {self.stats['total_files']}")
        self.logger.info(f"Files backed up: {self.stats['successful_files']}")
        self.logger.info(f"Failed files: {self.stats['failed_files']}")
        self.logger.info(f"New files: {self.stats['new_files']}")
        self.logger.info(f"Changed files: {self.stats['changed_files']}")
        self.logger.info(f"Unchanged files: {self.stats['unchanged_files']}")
        
        # Format total bytes
        bytes_transferred = self.stats["total_bytes"]
        if bytes_transferred < 1024:
            size_str = f"{bytes_transferred} bytes"
        elif bytes_transferred < 1024 * 1024:
            size_str = f"{bytes_transferred / 1024:.2f} KB"
        else:
            size_str = f"{bytes_transferred / (1024 * 1024):.2f} MB"
        
        self.logger.info(f"Total data transferred: {size_str}")
        
        # Calculate transfer rate
        if duration_seconds > 0 and bytes_transferred > 0:
            rate = bytes_transferred / duration_seconds
            if rate < 1024:
                rate_str = f"{rate:.2f} bytes/sec"
            elif rate < 1024 * 1024:
                rate_str = f"{rate / 1024:.2f} KB/sec"
            else:
                rate_str = f"{rate / (1024 * 1024):.2f} MB/sec"
            
            self.logger.info(f"Transfer rate: {rate_str}")
        
        self.logger.info("=" * 50)
    
    async def perform_backup(self):
        """Perform a complete backup operation."""
        # Reset stats for this run
        self.stats = {
            "start_time": datetime.now(),
            "end_time": None,
            "total_files": 0,
            "successful_files": 0,
            "failed_files": 0,
            "new_files": 0,
            "changed_files": 0,
            "unchanged_files": 0,
            "total_bytes": 0
        }
        
        try:
            # Connect to the RoboRio
            if not await self.connect_with_retry():
                return False
            
            # Create backup directory
            self.create_backup_dir()
            
            # Backup regular files
            await self.backup_regular_files()
            
            # Backup wpilog files
            await self.backup_wpilog_files()
            
            # Update end time
            self.stats["end_time"] = datetime.now()
            
            # Print summary
            self.print_backup_summary()
            
            self.logger.info(f"Backup completed successfully to {self.current_backup_dir}")
            return True
            
        except Exception as e:
            self.logger.error(f"Backup failed: {e}")
            self.logger.debug(traceback.format_exc())
            return False
        finally:
            # Update end time if not already set
            if not self.stats["end_time"]:
                self.stats["end_time"] = datetime.now()
            
            # Close connections
            if self.sftp:
                self.sftp.exit()
            if self.conn:
                self.conn.close()
                self.logger.debug("SSH connection closed")

async def main():
    """Main entry point for the script."""
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Backup RoboRio files to Raspberry Pi")
    parser.add_argument("--address", default="10.80.89.2", help="RoboRio IP address")
    parser.add_argument("--user", default="lvuser", help="SSH username")
    parser.add_argument("--backup-dir", default="./backups", help="Backup directory")
    parser.add_argument("--interval", type=int, help="Run continuously with specified interval (seconds)")
    parser.add_argument("--retries", type=int, default=3, help="Maximum connection retry attempts (default %(default)s)")
    parser.add_argument("--retry-delay", type=int, default=5, help="Delay between retry attempts (default %(default)ss")
    parser.add_argument("-v", "--verbose", action="count", default=0, help="Increase verbosity")
    args = parser.parse_args()
    
    # Configure logging
    log_level = logging.WARNING
    if args.verbose == 1:
        log_level = logging.INFO
    elif args.verbose >= 2:
        log_level = logging.DEBUG
    
    logging.basicConfig(
        level=log_level,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        stream=sys.stdout
    )
    
    logger = logging.getLogger("RioBackupMain")
    
    # Create the backup handler
    backup_handler = RioBackup(
        args.address, 
        args.user, 
        args.backup_dir,
        args.retries,
        args.retry_delay
    )
    
    if args.interval:
        # Run continuously with specified interval
        logger.info(f"Running in continuous mode with {args.interval} second interval")
        while True:
            try:
                await backup_handler.perform_backup()
            except Exception as e:
                logger.error(f"Unexpected error during backup: {e}")
                logger.debug(traceback.format_exc())
            
            logger.info(f"Waiting {args.interval} seconds until next backup...")
            await asyncio.sleep(args.interval)
    else:
        # Run once
        success = await backup_handler.perform_backup()
        return 0 if success else 1

if __name__ == "__main__":
    try:
        exit_code = asyncio.run(main())
        exit(exit_code)
    except KeyboardInterrupt:
        print("\nBackup interrupted by user. Exiting...")
        exit(1)
