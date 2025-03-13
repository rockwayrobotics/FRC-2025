#!/usr/bin/env python3

# TODO: camera calibration https://learnopencv.com/camera-calibration-using-opencv/
# see also https://github.com/tobybreckon/python-examples-cv/blob/master/calibrate_camera.py
# chessboard: https://docs.opencv.org/3.4/pattern.png

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
from cscore import CvSource, VideoMode, CameraServer, MjpegServer

import numpy as np
import numpy.typing

import cv2
import robotpy_apriltag as at

from ntcore import NetworkTableInstance, PubSubOptions
from wpimath.geometry import CoordinateSystem, Transform3d, Translation3d, Rotation3d, Pose3d
from wpimath import units

TAG_SIZE_METERS = 0.1651


class FieldTag:
    def __init__(self, id, absolutePose, corners):
        self.id = id
        self.absolutePose = absolutePose
        self.corners = corners # opencv coordinate system

class AprilTagDetection:
    def __init__(self, args, nt, cal):
        self.det = at.AprilTagDetector()
        self.det.addFamily('tag36h11') # default: bitsCorrected=2
        cfg = self.det.getConfig()
        cfg.quadDecimate = args.dec
        cfg.numThreads = args.threads
        cfg.decodeSharpening = 0.25 # margin jumps a lot with 1.0
        # cfg.quadSigma = 0.8
        self.det.setConfig(cfg)
        # print(f'Apriltags:\n\t{cfg.decodeSharpening=} {cfg.numThreads=} {cfg.quadDecimate=}\n'
        #     f'\t{cfg.quadSigma=} {cfg.refineEdges=} {cfg.debug=}')
        # q = det.getQuadThresholdParameters()
        # print(f'Quad Threshold:\n\t{degrees(q.criticalAngle)=} {q.deglitch=} {q.maxLineFitMSE=}\n'
        #     f'\t{q.maxNumMaxima=} {q.minClusterPixels=} {q.minWhiteBlackDiff=}')

        self.field = at.AprilTagFieldLayout("2025-reefscape.json")
        self.tags = {}
        for tag in self.field.getTags():
            absolutePose = tag.pose
            object_corners = compute_apriltag_corners(absolutePose)
            opencv_coordinate_object_corners = [wpilibTranslationToOpenCv(corner.translation()) for corner in object_corners]
            self.tags[tag.ID] = FieldTag(tag.ID, absolutePose, opencv_coordinate_object_corners)

        self.cal = cal
        self.cameraPose = nt.getFloatArrayTopic(
            "/Pi/Vision/cameraPose"
        ).publish(PubSubOptions())  # Uses default options

    def detect(self, arr):
        # img = arr[:height,:]
        img = cv2.cvtColor(arr, cv2.COLOR_RGB2GRAY)
        tags = self.det.detect(img)

        if bool(tags):
            object_corners = []
            image_corners = []
            for tag in tags:
                fieldTag = self.tags[tag.getId()]
                for i in range(4):
                    corner = tag.getCorner(i)
                    image_corners += [[corner.x, corner.y]]
                print(image_corners)
                shortest_side_length = min(
                    math.dist(image_corners[1], image_corners[0]),
                    math.dist(image_corners[2], image_corners[1]),
                )
                object_corners += fieldTag.corners
                print(object_corners)
                print(f"tag = {tag.getId()}")
                print(f"  image_corners = {image_corners}")
                print(f"  object_corners = {object_corners}")
                print(f"  shortest side: {shortest_side_length / 8}")

            _, rotations, translations, errors = cv2.solvePnPGeneric(
                np.array(object_corners),
                np.array(image_corners),
                self.cal.as_array(),
                None,
                flags=cv2.SOLVEPNP_SQPNP
            )
            print('opencv errors:', errors)

            camera_to_field_transform = openCvPoseToWpilib(translations[0], rotations[0])
            field_to_camera_transform = camera_to_field_transform.inverse()
            field_to_camera_pose = Pose3d(field_to_camera_transform.translation(), field_to_camera_transform.rotation())
            print(f'from opencv: {field_to_camera_pose}')
            pose1 = field_to_camera_pose.toPose2d()
            self.cameraPose.set([pose1.x, pose1.y, pose1.rotation().radians()])
            return True
        return False

def compute_apriltag_corners(tag_pose: Pose3d) -> list[Pose3d]:
    half_side_length = TAG_SIZE_METERS / 2
    # Bottom left, bottom right, top right, top left
    # WPILib's coorinates are north-west-up, opencv's are east-down-north
    return [
        tag_pose.transformBy(Transform3d(0, -half_side_length, -half_side_length, Rotation3d())),
        tag_pose.transformBy(Transform3d(0, half_side_length, -half_side_length, Rotation3d())),
        tag_pose.transformBy(Transform3d(0, half_side_length, half_side_length, Rotation3d())),
        tag_pose.transformBy(Transform3d(0, -half_side_length, half_side_length, Rotation3d())),
    ]

# Coordinate conversions inspired by
# https://github.com/Mechanical-Advantage/RobotCode2025Public/blob/8cd2135a6d7ee105b9f7596bb6261e5d611f4c91/northstar/pipeline/coordinate_systems.py
# Converting from East-Down-North (opencv's coordinates) to North-West-Up (wpilib's coordinates)
def openCvPoseToWpilib(tvec: np.typing.NDArray[np.float64], rvec: np.typing.NDArray[np.float64]) -> Transform3d:
    translation_edn, rotation_edn = tvec[:, 0], rvec[:, 0]
    tagInCameraFrame = Transform3d(
        Translation3d(*translation_edn),
        Rotation3d(rotation_edn)
    )
    return CoordinateSystem.convert(tagInCameraFrame, CoordinateSystem.EDN(), CoordinateSystem.NWU())


# Converting from North-West-Up (wpilib's coordinates) to East-Down-North (opencv's coordinates)
def wpilibTranslationToOpenCv(translation: Translation3d) -> list[float]:
    return [-translation.Y(), -translation.Z(), translation.X()]

def dict_to_opencv_camera_matrix(val: dict) -> np.typing.NDArray[float]:
    return np.array([
        val['fx'], 0, val['cx'],
        0, val['fy'], val['cy'],
        0, 0, 1
    ]).reshape((3, 3))

