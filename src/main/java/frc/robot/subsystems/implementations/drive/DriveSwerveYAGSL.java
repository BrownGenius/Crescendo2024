package frc.robot.subsystems.implementations.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.io.interfaces.DriveIO;
import frc.robot.io.interfaces.DriveIOInputsAutoLogged;
import frc.robot.subsystems.interfaces.Drive;
import java.io.File;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

public class DriveSwerveYAGSL extends DriveBase {
  public static class Constants {
    public static double slewRateLimiterX = 3;
    public static double slewRateLimiterY = 3;
    public static double slewRateLimiterAngle = 3;
  }

  private final File swerveJsonDirectory;
  private SwerveDrive swerveDrive;
  @AutoLogOutput private boolean fieldOrientedDrive = false;

  // @AutoLogOutput
  DriveIO io = new DriveIO();
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

  public DriveSwerveYAGSL(String configPath) {
    swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), configPath);

    // SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive =
          new SwerveParser(swerveJsonDirectory)
              .createSwerveDrive(Drive.Constants.maxVelocityMetersPerSec);
      swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);

      swerveDrive
          .getSwerveController()
          .addSlewRateLimiters(
              new SlewRateLimiter(Constants.slewRateLimiterX),
              new SlewRateLimiter(Constants.slewRateLimiterY),
              new SlewRateLimiter(Constants.slewRateLimiterAngle));

      swerveDrive.setMotorIdleMode(true);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      AutoBuilder.configure(
          swerveDrive::getPose, // Robot pose supplier
          swerveDrive::resetOdometry, // Method to reset odometry (will be called if your auto has a
          // starting pose)
          swerveDrive::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward) {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
            } else {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally
          // outputs individual module feedforwards
          new PPHolonomicDriveController( // PPHolonomicController is the built in path following
              // controller for holonomic drive trains
              new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
              ), // PPLTVController is the built in path following controller for differential drive
          // trains
          config, // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this // Reference to this subsystem to set requirements
          );
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
  }

  @Override
  public void runVelocity(ChassisSpeeds velocity) {
    if (fieldOrientedDrive) {
      swerveDrive.driveFieldOriented(velocity);
    } else {
      swerveDrive.drive(velocity);
    }
  }

  @Override
  public double getMaxLinearSpeed() {
    return swerveDrive.getMaximumChassisVelocity();
  }

  @Override
  public double getMaxAngularSpeed() {
    return swerveDrive.getMaximumChassisAngularVelocity();
  }

  @Override
  public void setFieldOrientedDrive(boolean enable) {
    fieldOrientedDrive = enable;
  }

  @Override
  public boolean isFieldOrientedDrive() {
    return fieldOrientedDrive;
  }

  @Override
  public void resetOdometry() {
    swerveDrive.resetOdometry(new Pose2d());
  }

  @Override
  public void setPoseToMatchField() {
    swerveDrive.resetOdometry(swerveDrive.field.getRobotPose());
  }

  /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return SysId Drive Command
   */
  public Command sysIdDriveMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(new Config(), this, swerveDrive, 12, false),
        3.0,
        5.0,
        3.0);
  }

  /**
   * Command to characterize the robot angle motors using SysId
   *
   * @return SysId Angle Command
   */
  public Command sysIdAngleMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(new Config(), this, swerveDrive), 3.0, 5.0, 3.0);
  }

  @Override
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  @Override
  public double getAngle() {
    return swerveDrive.getOdometryHeading().getDegrees();
  }

  @Override
  public void lockPose() {
    swerveDrive.lockPose();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs, swerveDrive);
    Logger.processInputs("Drive", inputs);
  }

  @Override
  public void addVisionMeasurement(
      Pose2d robotPose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {

    swerveDrive.addVisionMeasurement(robotPose, timestamp, visionMeasurementStdDevs);
  }
}
