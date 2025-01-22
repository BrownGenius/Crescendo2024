package frc.robot.subsystems.interfaces;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.text.DecimalFormat;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public interface Vision {
  @FunctionalInterface
  interface VisionMeasurementConsumer {
    void add(Pose2d robotPose, double timestamp, Matrix<N3, N1> stdDevs);
  }

  public class Camera {
    final String name;
    /*
     * Location of the camera view relative to the robot (in meters)
     *
     * Translation(x,y,z) - location of the camera relative to the center of the robot (in meters)
     *    x +/- is forward/back from center
     *    y +/- is left/right from center.
     *    z is relative to the ground (and should be positive)
     *  E.g. Translation(0.0, -0.5, 0.5) is 1/2 meter right of center and 1/2 meter above the ground
     *
     * Rotation(roll, pitch, yaw) - orientation of the camera view (in radians)
     *    roll +/- is CCW/CW around x-axis (should generally be 0)
     *    pitch +/- is down/up around y-axis (should generally be <= 0)
     *    yaw +/- is left/right around the z-axis
     *  E.g. Rotation(0, Units.degreesToRadians(-20), Units.degreesToRadians(-90)) is pitched up by 20 degrees, and facing to the right
     */
    final Transform3d robotToCamera;
    final String port;

    /**
     * @param cameraName
     * @param robotToCamera
     * @param port
     */
    public Camera(String cameraName, String port, Transform3d robotToCamera) {
      this.name = cameraName;
      this.port = port;
      this.robotToCamera = robotToCamera;
    }

    public Camera(String cameraName, Transform3d robotToCamera) {
      this(cameraName, null, robotToCamera);
    }

    public String getName() {
      return name;
    }

    public String getPort() {
      return port;
    }

    public Transform3d getRobotToCamera() {
      return robotToCamera;
    }
  }

  public class Pose {
    public static DecimalFormat doubleFormat = new DecimalFormat("0.00");
    public String cameraName;
    public Pose2d robotPose;
    public double timestamp;

    //    Matrix<N3,N1> visionMeasurementStdDevs; // Vision measurement standard deviation that will
    // be sent to the SwerveDrivePoseEstimator.The standard deviation of the vision measurement, for
    // best accuracy calculate the standard deviation at 2 or more points and fit a line to it with
    // the calculated optimal standard deviation. (Units should be meters per pixel). By optimizing
    // this you can get * vision accurate to inches instead of feet.

    /**
     * @param robotPose // Robot Pose2d as measured by vision
     * @param timestamp // Timestamp the measurement was taken as time since startup, should be
     *     taken from Timer.getFPGATimestamp() or similar sources.
     * @param cameraName // Name of camera this pose was calculated from
     */
    public Pose(Pose2d robotPose, double timestamp, String cameraName) {
      this.robotPose = robotPose;
      this.timestamp = timestamp;
      this.cameraName = cameraName;
    }

    public Pose() {
      this(new Pose2d(), -1, "");
    }

    @Override
    public String toString() {
      return "timestamp:"
          + Pose.doubleFormat.format(timestamp)
          + " cameraName:"
          + cameraName
          + " pose2d:(x:"
          + Pose.doubleFormat.format(robotPose.getX())
          + " y: "
          + Pose.doubleFormat.format(robotPose.getY())
          + " yaw: "
          + Pose.doubleFormat.format(robotPose.getRotation().getDegrees())
          + ")";
    }
  }

  public default boolean setPrimaryCamera(String name) {
    return true;
  }

  public Optional<Integer> getBestTargetId();

  public Optional<Double> getDistanceToBestTarget();

  /**
   * Returns the yaw in degrees to the best target (relative to the primary camera)
   *
   * @return yaw to the best target (in degrees)
   */
  public Optional<Double> getYawToBestTarget();

  /**
   * Returns the distance to the specified april tag in meters (relative to the primary camera)
   *
   * @param id AprilTag ID
   * @return distance to the specified april tag (in meters).
   */
  public Optional<Double> getDistanceToAprilTag(int id);

  /**
   * Returns the yaw in degrees to the specified april tag in meters (relative to the primary
   * camera)
   *
   * @param id AprilTag ID
   * @return yaw to the specified april tag (in degrees).
   */
  public Optional<Double> getYawToAprilTag(int id);

  public default void enableSimulation(Supplier<Pose2d> poseSupplier, boolean enableWireFrame) {}

  public default void setVisionMeasurementConsumer(VisionMeasurementConsumer func) {}

  public void addCamera(Camera camera);

  public List<Camera> getCameras();
}
