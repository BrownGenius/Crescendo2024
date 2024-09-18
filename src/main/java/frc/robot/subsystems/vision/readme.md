# Useful Resources:
* [PhotonVision](https://docs.photonvision.org/en/latest/)
* [Crescendo 2024 Vision System Design](https://docs.google.com/document/d/1OyC_vcDjkND8d1BjjKcrntKLCYnFIfZ_LF4Lyaiwpdk)

1. Initial Setup
    1. For each camera
        * For each resolution
            * [Calibrate](https://docs.photonvision.org/en/latest/docs/calibration/calibration.html) FOV
        * Relative to the center of the robot (See Transform3D [VisionCamera.VisionCamera](./VisionCamera.java))
            * Measure translation (x,y,z)
            * Measure rotation (roll, pitch, yaw)
    1. For each field
        * Load the AprilTag map layout into PhotonVision (See AprilTagFields.class)
2. Periodically at runtime
    1. For each camera
        * If estimated robot pose is available (i.e. AprilTag is visible)
            * addVisionMeasurement to drivetrain odometry (See [Drive.addVisionMeasurement()](../drive/Drive.java))