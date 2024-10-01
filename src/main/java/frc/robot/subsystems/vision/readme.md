# Useful Resources:
* [PhotonVision](https://docs.photonvision.org/en/latest/)
* [Crescendo 2024 Vision System Design](https://docs.google.com/document/d/1OyC_vcDjkND8d1BjjKcrntKLCYnFIfZ_LF4Lyaiwpdk)

1. Initial Setup
    1. For each camera
        * On the PhotonVision co-processor (e.g.. Raspberry Pi)
            * Assign a unique camera name
            * [2D AprilTag tracking](https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/2D-tracking-tuning.html) will work out of the box
                * You can get camera relative yaw, pitch, and roll information for each visible/detected AprilTag w/o calibrating the camera
            * [3D AprilTag tracking](https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/3D-tracking.html) requires calibration for each resolution:
                * [Calibrate](https://docs.photonvision.org/en/latest/docs/calibration/calibration.html) FOV
                * Measure location of camera relative to the center of the robot (See Transform3D [VisionCamera.VisionCamera](./VisionCamera.java))
                    * Measure translation (x,y,z)
                    * Measure rotation (roll, pitch, yaw)
    1. For each field
        * Load the AprilTag map layout into PhotonVision (See AprilTagFields.class)
2. Periodically at runtime
    1. For each camera
        * If estimated robot pose is available (i.e. AprilTag is visible and 3D tracking is enabled)
            * addVisionMeasurement to drivetrain odometry (See [Drive.addVisionMeasurement()](../drive/Drive.java))
