#! /usr/bin/env python3
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import os
import numpy as np
import array
from collections import defaultdict
import struct
from typing import List, SupportsBytes

import msgpack

__all__ = ["StartRecordData", "MetadataRecordData", "DataLogRecord", "DataLogReader"]

floatStruct = struct.Struct("<f")
doubleStruct = struct.Struct("<d")

kControlStart = 0
kControlFinish = 1
kControlSetMetadata = 2


class StartRecordData:
    """Data contained in a start control record as created by DataLog.start() when
    writing the log. This can be read by calling DataLogRecord.getStartData().

    entry: Entry ID; this will be used for this entry in future records.
    name: Entry name.
    type: Type of the stored data for this entry, as a string, e.g. "double".
    metadata: Initial metadata.
    """

    def __init__(self, entry: int, name: str, type: str, metadata: str):
        self.entry = entry
        self.name = name
        self.type = type
        self.metadata = metadata


class MetadataRecordData:
    """Data contained in a set metadata control record as created by
    DataLog.setMetadata(). This can be read by calling
    DataLogRecord.getSetMetadataData().

    entry: Entry ID.
    metadata: New metadata for the entry.
    """

    def __init__(self, entry: int, metadata: str):
        self.entry = entry
        self.metadata = metadata


class DataLogRecord:
    """A record in the data log. May represent either a control record
    (entry == 0) or a data record."""

    def __init__(self, entry: int, timestamp: int, data: SupportsBytes):
        self.entry = entry
        self.timestamp = timestamp
        self.data = data

    def isControl(self) -> bool:
        return self.entry == 0

    def _getControlType(self) -> int:
        return self.data[0]

    def isStart(self) -> bool:
        return (
            self.entry == 0
            and len(self.data) >= 17
            and self._getControlType() == kControlStart
        )

    def isFinish(self) -> bool:
        return (
            self.entry == 0
            and len(self.data) == 5
            and self._getControlType() == kControlFinish
        )

    def isSetMetadata(self) -> bool:
        return (
            self.entry == 0
            and len(self.data) >= 9
            and self._getControlType() == kControlSetMetadata
        )

    def getStartData(self) -> StartRecordData:
        if not self.isStart():
            raise TypeError("not a start record")
        entry = int.from_bytes(self.data[1:5], byteorder="little", signed=False)
        name, pos = self._readInnerString(5)
        type, pos = self._readInnerString(pos)
        metadata = self._readInnerString(pos)[0]
        return StartRecordData(entry, name, type, metadata)

    def getFinishEntry(self) -> int:
        if not self.isFinish():
            raise TypeError("not a finish record")
        return int.from_bytes(self.data[1:5], byteorder="little", signed=False)

    def getSetMetadataData(self) -> MetadataRecordData:
        if not self.isSetMetadata():
            raise TypeError("not a finish record")
        entry = int.from_bytes(self.data[1:5], byteorder="little", signed=False)
        metadata = self._readInnerString(5)[0]
        return MetadataRecordData(entry, metadata)

    def getBoolean(self) -> bool:
        if len(self.data) != 1:
            raise TypeError("not a boolean")
        return self.data[0] != 0

    def getInteger(self) -> int:
        if len(self.data) != 8:
            raise TypeError("not an integer")
        return int.from_bytes(self.data, byteorder="little", signed=True)

    def getFloat(self) -> float:
        if len(self.data) != 4:
            raise TypeError("not a float")
        return floatStruct.unpack(self.data)[0]

    def getDouble(self) -> float:
        if len(self.data) != 8:
            raise TypeError("not a double")
        return doubleStruct.unpack(self.data)[0]

    def getString(self) -> str:
        return str(self.data, encoding="utf-8")

    def getMsgPack(self):
        return msgpack.unpackb(self.data)

    def getBooleanArray(self) -> List[bool]:
        return [x != 0 for x in self.data]

    def getIntegerArray(self) -> array.array:
        if (len(self.data) % 8) != 0:
            raise TypeError("not an integer array")
        arr = array.array("l")
        arr.frombytes(self.data)
        return arr

    def getFloatArray(self) -> array.array:
        if (len(self.data) % 4) != 0:
            raise TypeError("not a float array")
        arr = array.array("f")
        arr.frombytes(self.data)
        return arr

    def getDoubleArray(self) -> array.array:
        if (len(self.data) % 8) != 0:
            raise TypeError("not a double array")
        arr = array.array("d")
        arr.frombytes(self.data)
        return arr

    def getStringArray(self) -> List[str]:
        size = int.from_bytes(self.data[:4], byteorder="little", signed=False)
        if size > ((len(self.data) - 4) / 4):
            raise TypeError("not a string array")
        arr = []
        pos = 4
        for _ in range(size):
            val, pos = self._readInnerString(pos)
            arr.append(val)
        return arr

    def _readInnerString(self, pos: int) -> tuple[str, int]:
        size = int.from_bytes(
            self.data[pos : pos + 4], byteorder="little", signed=False
        )
        end = pos + 4 + size
        if end > len(self.data):
            raise TypeError("invalid string size")
        return str(self.data[pos + 4 : end], encoding="utf-8"), end


class DataLogIterator:
    """DataLogReader iterator."""

    def __init__(self, buf: SupportsBytes, pos: int):
        self.buf = buf
        self.pos = pos

    def __iter__(self):
        return self

    def _readVarInt(self, pos: int, len: int) -> int:
        val = 0
        for i in range(len):
            val |= self.buf[pos + i] << (i * 8)
        return val

    def __next__(self) -> DataLogRecord:
        if len(self.buf) < (self.pos + 4):
            raise StopIteration
        entryLen = (self.buf[self.pos] & 0x3) + 1
        sizeLen = ((self.buf[self.pos] >> 2) & 0x3) + 1
        timestampLen = ((self.buf[self.pos] >> 4) & 0x7) + 1
        headerLen = 1 + entryLen + sizeLen + timestampLen
        if len(self.buf) < (self.pos + headerLen):
            raise StopIteration
        entry = self._readVarInt(self.pos + 1, entryLen)
        size = self._readVarInt(self.pos + 1 + entryLen, sizeLen)
        timestamp = self._readVarInt(self.pos + 1 + entryLen + sizeLen, timestampLen)
        if len(self.buf) < (self.pos + headerLen + size):
            raise StopIteration
        record = DataLogRecord(
            entry,
            timestamp,
            self.buf[self.pos + headerLen : self.pos + headerLen + size],
        )
        self.pos += headerLen + size
        return record


class DataLogReader:
    """Data log reader (reads logs written by the DataLog class)."""

    def __init__(self, buf: SupportsBytes):
        self.buf = buf

    def __bool__(self):
        return self.isValid()

    def isValid(self) -> bool:
        """Returns true if the data log is valid (e.g. has a valid header)."""
        return (
            len(self.buf) >= 12
            and self.buf[:6] == b"WPILOG"
            and self.getVersion() >= 0x0100
        )

    def getVersion(self) -> int:
        """Gets the data log version. Returns 0 if data log is invalid.

        @return Version number; most significant byte is major, least significant is
            minor (so version 1.0 will be 0x0100)"""
        if len(self.buf) < 12:
            return 0
        return int.from_bytes(self.buf[6:8], byteorder="little", signed=False)

    def getExtraHeader(self) -> str:
        """Gets the extra header data.

        @return Extra header data
        """
        if len(self.buf) < 12:
            return ""
        size = int.from_bytes(self.buf[8:12], byteorder="little", signed=False)
        return str(self.buf[12 : 12 + size], encoding="utf-8")

    def __iter__(self) -> DataLogIterator:
        extraHeaderSize = int.from_bytes(
            self.buf[8:12], byteorder="little", signed=False
        )
        return DataLogIterator(self.buf, 12 + extraHeaderSize)


COLS = [
    "yaw",
    "left_pos",
    "right_pos",
    "left_vel",
    "right_vel",
    "auto",
    "enabled",
    "bb_irq_ts",
    "corner_pos",
    "corner_send_ts",
    "corner_detect_ts",
    "corner_detect_nt_ts",
    "tof_first_ts",
    "tof_ts",
    "tof_dist",
    "tof_nt_ts",
    "tof_fpga_ts",
    "shoot_counter",
]

MAPPINGS = {
    "/Drive/Gyro/YawPosition": "yaw",
    "/Drive/LeftPositionMeters": "left_pos",
    "/Drive/RightPositionMeters": "right_pos",
    "/Drive/LeftVelocityMetersPerSec": "left_vel",
    "/Drive/RightVelocityMetersPerSec": "right_vel",
    "/DriverStation/Autonomous": "auto",
    "/DriverStation/Enabled": "enabled",
    "/RealOutputs/DigitalIO/Falling": "bb_irq_ts",
    "/RealOutputs/ToF/CornerPosition": "corner_pos",
    "/RealOutputs/ToF/Corners": {
        0: "corner_send_ts",
        1: "corner_detect_ts",
        2: "corner_detect_nt_ts",
    },
    "/RealOutputs/ToF/Everything": {
        0: "tof_first_ts",
        2: "tof_ts",
        3: "tof_dist",
        5: "tof_nt_ts",
        6: "tof_fpga_ts",
    },
    "/RealOutputs/ToF/ShootCounter": "shoot_counter",

    # Older wpilogs had these separated
    "/RealOutputs/ToF/Distance": "tof_dist",
    "/RealOutputs/ToF/FPGA": "tof_fpga_ts",
    "/RealOutputs/ToF/NT_Time": "tof_nt_ts",
    "/RealOutputs/ToF/Timestamps": {
        0: "tof_first_ts",
        2: "tof_ts",
    },
}

# clean data for CSV output
def clean(v):
    v = v.replace('"', '""')
    if "," in v:
        v = f'"{v}"'
    return v

def start_csv_file(csvname):
    csvfile = open(csvname, "w")
    print(",".join(["time"] + COLS), file=csvfile)
    return csvfile

def parse_file(input, prefix, tomlfile):
    with open(input, "r") as f:
        mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
        reader = DataLogReader(mm)
        if not reader:
            print("not a log file", file=sys.stderr)
            sys.exit(1)

        approach_count = 1
        entries = {}
        auto = False
        enabled = False
        writing_csv = False
        corner_found = False
        distances = np.zeros((1000, 2), dtype="f")
        for i, record in enumerate(reader):
            # if i > 1e6 + 10000:
            #     break
            timestamp = record.timestamp / 1000000
            # breakpoint()
            if record.isStart():
                try:
                    data = record.getStartData()
                    # print(
                    #     f"Start({data.entry}, name='{data.name}', type='{data.type}', metadata='{data.metadata}') [{timestamp}]"
                    # )
                    # if data.entry in entries:
                    #     print("...DUPLICATE entry ID, overriding")
                    entries[data.entry] = data
                    row = [record.timestamp, {}]
                except TypeError:
                    print("Start(INVALID)")
            elif record.isFinish():
                try:
                    entry = record.getFinishEntry()
                    print(f"Finish({entry}) [{timestamp}]")
                    if entry not in entries:
                        print("...ID not found")
                    else:
                        del entries[entry]
                except TypeError:
                    print("Finish(INVALID)")
            elif record.isSetMetadata():
                try:
                    data = record.getSetMetadataData()
                    print(f"SetMetadata({data.entry}, '{data.metadata}') [{timestamp}]")
                    if data.entry not in entries:
                        print("...ID not found")
                except TypeError:
                    print("SetMetadata(INVALID)")
            elif record.isControl():
                print("Unrecognized control record")
            else:
                # if i < 1e6:
                #     continue
                entry = entries.get(record.entry)
                # print(f"Data({record.entry}, size={len(record.data)}) ", end="")
                if entry is None:
                    # print("<ID not found>")
                    continue
                # print(f"<name='{entry.name}', type='{entry.type}'> [{timestamp}]")
                if entry.name not in MAPPINGS:
                    continue
                mapping = MAPPINGS[entry.name]

                if row[0] != record.timestamp:
                    cols = ",".join(clean(row[1].get(k, "")) for k in COLS).rstrip(",")
                    if auto and enabled and not stopped:
                        if not writing_csv:
                            csvfile = start_csv_file(f"{prefix}_{approach_count}.csv")
                            print(f"\n[approach{approach_count}]", file=tomlfile)
                            writing_csv = True
                            corner_found = False
                            distance_count = 0
                            approach_count += 1
                        print(f"{timestamp:.3f},{cols}", file=csvfile)
                    elif writing_csv:
                        writing_csv = False
                    # print(f"{row[0] / 1e6:.3f},{row[1]}")

                    row = [record.timestamp, {}]

                # print(f"{timestamp:.3f},{entry.name}")

                try:
                    # handle systemTime specially
                    if entry.name == "systemTime" and entry.type == "int64":
                        dt = datetime.fromtimestamp(record.getInteger() / 1000000)
                        print("  {:%Y-%m-%d %H:%M:%S.%f}".format(dt))
                        continue

                    if entry.type == "double":
                        if mapping == "left_vel" or mapping == "right_vel":
                            stopped = record.getDouble() < 0.0005
                        elif writing_csv:
                            if mapping == "bb_irq_ts":
                                print(f"bb_irq_ts={record.getDouble():.3f}", file=tomlfile)
                                # we know that the beam break is hit when we are on the main face
                                # so try to calculate the slope here to estimate our angle from the wall
                                # These are not helpful
                                # print("Distances:", distances[0:distance_count])
                                # print("Gradient:", np.gradient(distances[0:distance_count,1], distances[0:distance_count,0]))
                            elif mapping == "corner_pos":
                                if not corner_found:
                                    print(f"corner_pos={record.getDouble():.3f}", file=tomlfile)
                                    corner_found = True
                        value = f"{record.getDouble():.3f}"
                    elif entry.type == "int64":
                        value = f"{record.getInteger()}"
                        if mapping == "shoot_counter" and writing_csv:
                            print(f"shot_fpga_ts={timestamp:.3f}", file=tomlfile)
                    elif entry.type in ("string", "json"):
                        value = f"'{record.getString()}'"
                    elif entry.type == "msgpack":
                        value = f"'{record.getMsgPack()}'"
                    elif entry.type == "boolean":
                        value = f"{record.getBoolean()}"
                        if mapping == "auto":
                            auto = record.getBoolean()
                        elif mapping == "enabled":
                            enabled = record.getBoolean()
                    elif entry.type == "boolean[]":
                        arr = record.getBooleanArray()
                        value = f"{arr}"
                    elif entry.type == "double[]":
                        arr = record.getDoubleArray()
                        value = f"{arr}"
                    elif entry.type == "float[]":
                        arr = record.getFloatArray()
                        if type(mapping) is dict:
                            for key in mapping:
                                if writing_csv:
                                    if mapping[key] == "corner_detect_ts":
                                        print(f"corner_detect_ts={arr[key]:.3f}", file=tomlfile)
                                    elif mapping[key] == "corner_detect_nt_ts":
                                        print(f"corner_detect_nt_ts={arr[key]:.3f}", file=tomlfile)
                                    elif mapping[key] == "tof_dist":
                                        distances[distance_count][0] = timestamp
                                        distances[distance_count][1] = arr[key]
                                        distance_count += 1

                                if mapping[key] == "tof_dist":
                                    row[1][mapping[key]] = f"{arr[key]:.0f}"
                                else:
                                    row[1][mapping[key]] = f"{arr[key]:.3f}"
                        else:
                            value = f"{arr:.3f}"
                    elif entry.type == "int64[]":
                        arr = record.getIntegerArray()
                        value = f"{arr}"
                    elif entry.type == "string[]":
                        arr = record.getStringArray()
                        value = f"{arr}"
                    elif entry.type == "struct:Rotation2d":
                        # Structs are actually defined with a structschema message
                        # but that requires work to parse and store so we are ignoring it
                        value = f"{record.getDouble():.3f}"
                except TypeError:
                    value = "invalid"

                if type(mapping) is not dict:
                    row[1][mapping] = value

if __name__ == "__main__":
    import mmap
    import sys
    from datetime import datetime

    if len(sys.argv) != 3:
        print("Usage: tofparse.py <file> <output-prefix>", file=sys.stderr)
        sys.exit(1)

    tomlname = f"{sys.argv[2]}.toml"
    if os.path.exists(tomlname):
        print(f"Refusing to overwrite {tomlname}")
        sys.exit(3)
    
    with open(tomlname, 'w') as tomlfile:
        parse_file(sys.argv[1], sys.argv[2], tomlfile)
