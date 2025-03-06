import random
import time

from ntcore import NetworkTableInstance, PubSubOptions
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Pose2d

def main():
    nt = NetworkTableInstance.getDefault()
    pub1 = nt.getStructTopic(
        "/vision/pose2d",
        Pose2d, # Transform3d
    ).publish(PubSubOptions())  # Uses default options

    pub2 = nt.getIntegerTopic("/vision/test1").publish(PubSubOptions())

    pub3 = nt.getFloatArrayTopic(
        "/Shuffleboard/Drivebase/Field2d/Robot"
    ).publish(PubSubOptions())  # Uses default options

    nt.startServer()

    # While all our variants of publishing things worked

    while True:
        y = random.random() * 3 + 1.0
        # transform = Transform3d(
        #     Translation3d(1.0, y, 3.0),
        #     Rotation3d(0.1, 0.2, 0.3)
        # )
        # pub1.set(transform)
        # print(transform)
        val1 = Pose2d(1.1, y, 0.3)
        pub1.set(val1)

        val2 = random.randint(0, 2)
        pub2.set(val2)

        val3  = [random.random() * 10 + 0.5, y, random.random() * 180]
        pub3.set(val3)

        print(val1, val2, val3)

        time.sleep(random.random() * 0.5 + 0.016)


if __name__ == '__main__':
    main()
