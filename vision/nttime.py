import datetime as dt
import time

import ntcore
import wpiutil
import threading
import socket
import struct
import wpiutil.log as wlog


def udp_watcher(udp_trec):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", 3000))

    while True:
        data, addr = sock.recvfrom(1024)
        if len(data) == 8:
            now = time.monotonic()
            ts = struct.unpack("<Q", data[0:8])[0]
            udp_trec.append([now, ts / 1e6])
            print([now, ts / 1e6])


def main():
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.setServerTeam(8089)
    inst.startClient4("mypi")
    sub = inst.getDoubleTopic("/AdvantageKit/Timestamp").subscribe(0)

    times = []
    global enable  # I forget how to do the local one that's not global
    enable = False

    def on_pub(evt):
        global enable
        if enable:
            enable = False
            times.append((dt.datetime.now(), time.time(), time.monotonic(), evt.data.value))

    listener = inst.addListener(["/AdvantageKit/Timestamp"], ntcore.EventFlags.kValueAll, on_pub)

    ts = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    fname = f"nttime-{ts}.wpilog"
    dlog = wpiutil.DataLogBackgroundWriter("logs", fname, extraHeader="extraHeader: this is a test")
    trec = wlog.DoubleArrayLogEntry(dlog, "/Pi/sync_times")
    udp_trec = wlog.DoubleArrayLogEntry(dlog, "/Pi/sync_times_udp")

    thread = threading.Thread(target=udp_watcher, args=(udp_trec,))
    thread.daemon = True
    thread.start()

    base1 = time.time()
    base2 = time.monotonic()
    # datetime.now   :      time.mono         time.time             d1          evt time          value      server time
    # 19:58:19.459626:   11728.512182 1742255899.459637  -11728.309196 1742255899.416547 1742254157148919.250000  2793937762  2793937762            ts = now.
    print(
        "datetime.now        time.mono         time.time         evt-time         d1          d2       value server-time  d3"
    )
    while True:
        enable = True
        time.sleep(1)
        if times:
            val = times.pop(0)
            now = val[0]
            ttime = val[1]
            mono = val[2]
            evt = val[3]
            value = evt.getInteger()
            stime = evt.server_time()
            evttime = evt.time()
            d1 = evttime / 1e6 - ttime
            d2 = evttime / 1e6 - base1 + base2 - mono
            trec.append([mono, ttime, evttime / 1e6, d1, d2, value, stime])
            print(
                f"{now.time()} {mono:13.6f} {ttime:17.6f} {evttime/1e6:17.6f} {d1:10.6f} {d2:10.6f} {value:11} {stime:11}  {value-stime}"
            )

        time.sleep(10)


if __name__ == "__main__":
    main()
