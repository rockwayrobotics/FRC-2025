#!/usr/bin/env python3
'''NT time sync research utility.'''

import asyncio
import datetime as dt
import logging
# import math
import socket
import struct
import sys
import threading
import time

import ntcore
import wpiutil
import wpiutil.log as wlog

UDP_PORT = 3000

class TimeMonitor:
    def __init__(self, args):
        self.args = args
        self.log = logging.getLogger('tm')
        self.conn_state = None
        self.udp_offsets = []
        self.udp_offset = None
        self.st_offset = None
        self.sync = 0.0
        self.mono_base = time.monotonic()
        self.mono_adjust = time.time() - self.mono_base
        self.log.debug('mono_adjust %.6f', self.mono_adjust)

    def udp_watcher(self):
        log = logging.getLogger('udp')
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("", UDP_PORT))
        self.log.debug(f'UDP bound on port {UDP_PORT}')

        while True:
            data, addr = sock.recvfrom(1024)
            if len(data) == 8:
                mono = time.monotonic()
                fpga = struct.unpack("<Q", data[0:8])[0]
                # Since fpga was set on the Rio, the time delay between it
                # and when we record mono time will be slightly above zero
                # due to network latency.
                # ts is FPGA time (maybe with epoch added?) in microseconds
                msg = ('udp', (mono, fpga))
                self.loop.call_soon_threadsafe(self.msgs.put_nowait, msg)
                # if sys.stdout.isatty():
                #     self.log.debug('udp: mono=%.6f fpga=%d', mono, fpga)

    async def main(self):
        self.loop = asyncio.get_event_loop()
        self.msgs = asyncio.Queue()

        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.inst.setServerTeam(8089)
        self.inst.startClient4("nttime")

        self.ts_sub = self.inst.getDoubleTopic("/AdvantageKit/Timestamp").subscribe(0)
        # et_sub = self.inst.getDoubleTopic("/AdvantageKit/SystemStats/EpochTimeMicros").subscribe(0)

        # brief pause to let NT do time sync and grab initial values,
        # else we may read our own defaults
        # await asyncio.sleep(0.1)

        # ts_listener = inst.addListener(["/AdvantageKit/Timestamp"], ntcore.EventFlags.kValueAll, on_timestamp)
        conn_listener = self.inst.addConnectionListener(immediate_notify=True, callback=self.on_conn_event)

        ts = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
        fname = f"nttime-{ts}.wpilog"
        dlog = wpiutil.DataLogBackgroundWriter("logs", fname)
        # self.st_trec = wlog.DoubleArrayLogEntry(dlog, "/Pi/sync_times")
        self.udp_trec = wlog.DoubleLogEntry(dlog, "/Pi/udp_offset")
        self.st_trec = wlog.DoubleLogEntry(dlog, "/Pi/st_offset")

        self.fpga_trec = wlog.DoubleLogEntry(dlog, "/Pi/fpga_time")
        self.pi_rio_trec = wlog.DoubleArrayLogEntry(dlog, "/Pi/pi_rio")

        # clock delta as determined by comparing the UDP timestamp info
        # this value represents rate of (relative) drift in the clocks over time
        self.sync_trec = wlog.DoubleLogEntry(dlog, "/Pi/sync")

        # measured error in getServerTimeOffset() compared to what UDP info tells us
        self.error_trec = wlog.DoubleLogEntry(dlog, "/Pi/sync_error")

        thread = threading.Thread(target=self.udp_watcher, daemon=True)
        thread.start()

        try:
            await self.run()
        finally:
            self.inst.removeListener(conn_listener)
            self.log.info('exiting')

    def timestamp(self):
        return int((time.monotonic() - self.mono_base) * 1e6)
            
    async def run(self):
        # datetime.now   :      time.mono         time.time             d1          evt time          value      server time
        # 19:58:19.459626:   11728.512182 1742255899.459637  -11728.309196 1742255899.416547 1742254157148919.250000  2793937762  2793937762            ts = now.
        # print(
        #     "time.mono         time.time         evt-time         d1          d2       value server-time  d3"
        # )
        
        timeout = None

        self.update_st_offset(self.inst.getServerTimeOffset())
        self.recheck = None
        
        while True:
            msg = await self.msgs.get()
            
            now = time.monotonic()
            
            # handle messages
            try:
                tag, data = msg
            except Exception as ex:
                self.log.error('invalid msg: %r (%s)', msg, ex)
                continue

            match tag:
                case 'udp':
                    mono, fpga = data
                    # self.log.debug('udp: %.6f, FPGA= %.3f bits', mono, math.log2(fpga))
                    # Server time offset is such that adding it to
                    # local time maps the value to FPGA time.
                    # fpga = local + server_time_offset
                    # Therefore offset = fpga - local
                    # Note that our local time measurement is technically always
                    # slightly later than the true difference between these
                    # since it takes a few microseconds for the packet to travel
                    # the network.
                    self.update_udp_offset(ts, fpga / 1e6)

                    # fpga = ts_sub.get() / 1e6
                    offset = self.inst.getServerTimeOffset()
                    self.update_st_offset(offset)

                case 'conn':
                    self.update_conn_status(data.flags)
                    # for development:
                    global conn_event
                    conn_event = data

                case _:
                    self.log.warning('unhandled msg: %r', tag)

            if self.recheck is not None and now >= self.recheck:
                self.recheck = now + 60

                self.update_st_offset(self.inst.getServerTimeOffset())
                ts = self.ts_sub.getAtomic(0)
                mono = time.monotonic()
                # This is basically the magic value that maps the FPGA timestamp
                # to an epoch time. We hypothesize that it's set as a one-time
                # thing when the robot code starts up and/or gets synced with an
                # external time base.
                delta1 = (ts.time - ts.serverTime) / 1e6

                # In theory, something like this should reproduce it using
                # our own time deltas.
                delta2 = -self.udp_offset 
                error = delta2 - delta1
                now = self.timestamp()
                self.error_trec.append(int(error * 1e6), timestamp=now)
                self.fpga_trec.append(ts.serverTime, timestamp=now)
                if self.udp_offset is not None:
                    # ts - time.monotonic() + (ntcore._now() + offset_or_none) / 1e6
                    # ntcore._now() is local epoch time in microseconds (really big)
                    # server time offset is in microseconds too, and added to ntcore._now()
                    # (really big negative)
                    # will give us the FPGA timestamp (apparently 64-bit version)
                    rt1 = ntcore._now() /1e6 + self.st_offset
                    rt2 = mono + self.mono_adjust + self.udp_offset
                    self.pi_rio_trec.append([rt1, rt2, rt2-rt1], timestamp=now)

                    # check
                    self.log.info('RIO times: rt1=%.6f, rt2=%.6f, diff=%.6f', rt1, rt2, rt2-rt1)

                if sys.stdout.isatty():
                    self.log.debug('/Timestamp: %s,%s,%s delta1=%.6f delta2=%.6f error=%.6f',
                        ts.time, ts.serverTime, ts.value, delta1, delta2, error)
                    self.log.debug('mono_base %.6f, mono_adjust %.6f, udp %.6f, st %.6f',
                        self.mono_base, self.mono_adjust,
                        self.udp_offset, self.st_offset)


                
    def update_udp_offset(self, mono, fpga):
        '''Calculate offset between FPGA value sent over UDP and
        our local mono time.  Because of network latency the udp offset value
        will be lower than the "true" value, and more lower then larger the
        network latency.''''
        ts = mono + self.mono_adjust
        offset = fpga - ts

        # very crude first version, just remembering the last X values
        # and picking the minimum on the assumption that the only error
        # comes from network transit delays, which can only be positive
        self.udp_offsets.append(offset)
        SIZE = 30
        if len(self.udp_offsets) > SIZE:
            del self.udp_offsets[:-SIZE]

        # Since larger network delay makes the values lower, we want the
        # highest recent one.  We don't need to track the slope as the
        # rate of change is low enough that 30 seconds of data
        offset = max(self.udp_offsets)

        if self.udp_offset is not None:
            delta = offset - self.udp_offset
            if abs(delta) >= 0.000100:
                self.log.info('udp_offset changed: %.6f -> %.6f (delta %.6f)',
                    self.udp_offset, offset, delta)
                values = [int((x - offset) * 1e6) for x in self.udp_offsets]
                self.log.debug('values: %s', ','.join(f'{x}' for x in values))
        else:
            self.log.info('initial udp_offset: %.6f', offset)

        self.udp_offset = offset
        self.udp_trec.append(offset, timestamp=self.timestamp())
        self.update_sync(self.udp_offset, self.st_offset)
        
    def update_st_offset(self, offset):
        match (self.st_offset, offset):
            case (None, None):
                pass
            
            case (None, _):
                offset /= 1e6
                self.log.info('initial st_offset: %.6f', offset)
                self.update_sync(self.udp_offset, offset)

            case (_, None):
                self.log.warning('lost connection, st_offset: None')

            case (prev, offset):
                offset /= 1e6
                delta = offset - prev
                if abs(delta) > 0.0:
                    self.log.info('st_offset changed: %.6f -> %.6f (delta %.6f)',
                        prev, offset, delta)
                    self.update_sync(self.udp_offset, offset)

        self.st_offset = offset
        self.st_trec.append(offset or 0, timestamp=self.timestamp())

    def update_sync(self, udp_offset, st_offset):
        if udp_offset is None or st_offset is None:
            return
        
        now = time.monotonic()
        adj_udp_offset = udp_offset # hmm... what?
        sync = adj_udp_offset - st_offset
        if abs(sync - self.sync) >= 0.000100:
            self.log.info('%.3f: sync changed: %.6f -> %.6f (delta %.6f)',
                now - self.mono_base,
                self.sync, sync, (sync - self.sync))
            self.sync = sync

            # fpga1 = now + st_offset + self.mono_adjust
            # fpga2 = now + udp_offset + self.mono_adjust
            # self.log.debug('%.3f: fpga diff: %.6f (%.6f, %.6f)',
            #     now,
            #     fpga2 - fpga1, st_offset + self.mono_adjust, udp_offset + self.mono_adjust)

        self.sync_trec.append(int(sync * 1e6), timestamp=self.timestamp())

    # def on_timestamp(self, evt):
    #     msg = ('ts', (time.time(), time.monotonic(), evt.data.value))
    #     self.events.put_nowait(msg)

    def on_conn_event(self, evt):
        msg = ('conn', evt)
        self.loop.call_soon_threadsafe(self.msgs.put_nowait, msg)

    def update_conn_status(self, flags):
        if self.conn_state != flags:
            self.conn_state = flags
            # Note:
            #   flags 2 means connected
            #   flags 4 means disconnected
            self.log.info('conn: %s', flags)
            if flags & 4:
                self.udp_offsets = []
                self.st_offset = None
            elif flags & 2:
                self.recheck = time.monotonic() + 5.0

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--debug", action="store_true")

    args = parser.parse_args()

    level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(level=level,
        format="%(asctime)s.%(msecs)03d:%(levelname)5s:%(name)s: %(message)s",
        datefmt="%H:%M:%S",
        stream=sys.stdout,
    )

    tm = TimeMonitor(args)

    try:
        asyncio.run(tm.main())
    except (SystemExit, KeyboardInterrupt):
        pass
