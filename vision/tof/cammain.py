#!/usr/bin/env python3

# pylint: disable=unused-import,invalid-name,trailing-whitespace,missing-module-docstring
# pylint: disable=missing-function-docstring,missing-class-docstring,no-member
# pylint: disable=consider-using-with,too-few-public-methods

import datetime as dt
import io
import logging
import socketserver
from http import server
import threading
import math
import os
import platform
import time
import traceback

from ntcore import NetworkTableInstance, PubSubOptions


def main():
    args = get_args()
    res = args.res

    nt = NetworkTableInstance.getDefault()
    nt.setServerTeam(8089)
    nt.startClient4("tof")

    # mjpegTopic = nt.getStringArrayTopic("/CameraPublisher/PiCam/streams").publish(PubSubOptions())

    selectTopic = nt.getIntegerTopic("/Pi/select")
    select = selectTopic.publish(PubSubOptions())
    select.set(0)
    select = selectTopic.subscribe(0)

    # camAutoDeadbandTopic = nt.getDoubleTopic("/Tuning/Camera Auto Deadband")
    # camAutoDeadbandSub = camAutoDeadbandTopic.subscribe(0)    

    # rightVelocitySub = nt.getDoubleTopic("/AdvantageKit/Drive/RightVelocityMetersPerSec").subscribe(0)
    # leftVelocitySub = nt.getDoubleTopic("/AdvantageKit/Drive/LeftVelocityMetersPerSec").subscribe(0)

    # mjpegTopic.set(["mjpg:http://10.80.89.11:8081/?action=stream"])

    try:

    finally:

        print('exiting main')


def get_args():
    # pylint: disable=import-outside-toplevel
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--debug', action='store_true')

    # This is a bad way to do this, shoving these into the global
    # namespace, but it's a temporary hack to avoid having to change some
    # of the original code which did that as a shortcut.
    args = parser.parse_args()
    return args


if __name__ == '__main__':
    main()
