#!/usr/bin/env python3

import argparse
import asyncio
import logging

from .main import ResetService

def parse_args():
    """Parse command line arguments"""
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

if __name__ == "__main__":
    asyncio.run(main())
