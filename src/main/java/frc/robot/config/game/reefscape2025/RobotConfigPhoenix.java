package frc.robot.config.game.reefscape2025;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.game.crescendo2024.auto.AutoNamedCommands;
import frc.robot.config.RobotConfig;
import frc.robot.subsystems.implementations.arm.ArmSubsystem;
import frc.robot.subsystems.implementations.drive.DriveBase;
import frc.robot.subsystems.implementations.drive.DriveSwerveYAGSL;
import frc.robot.subsystems.interfaces.Drive;

/* Override Phoenix specific constants here */
public class RobotConfigPhoenix extends RobotConfig {
  public RobotConfigPhoenix() {
    super(false, true, true, true, true, true, true);

    ArmSubsystem.Constants.minDistanceInMeters = Units.inchesToMeters(38);
    ArmSubsystem.Constants.maxDistanceInMeters = 4.0;
    ArmSubsystem.Constants.Ax2 = -3.2;
    ArmSubsystem.Constants.Bx = 23.7;
    ArmSubsystem.Constants.C = -10.3;

    // Phoenix has a Swerve drive train
    Drive.Constants.rotatePidKp = 0.025;
    Drive.Constants.rotatePidKi = 0.0;
    Drive.Constants.rotatePidKd = 0.0;
    DriveBase.Constants.rotatePidErrorInDegrees = 1;
    drive = new DriveSwerveYAGSL("yagsl/phoenix");

    // cameras = new ArrayList<VisionCamera>();
    // cameras.add(
    //     new VisionCamera(
    //         "shooter",
    //         "1182",
    //         new Transform3d(
    //             new Translation3d(-Units.inchesToMeters(10.75), 0, Units.inchesToMeters(8)),
    //             new Rotation3d(0, Units.degreesToRadians(-33), Units.degreesToRadians(180)))));

    // VisionConstants.visionDistanceOffsetInMeters = -0.2;
    // vision = new VisionSubsystem(cameras,
    // AprilTagFields.k2024Crescendo.loadAprilTagLayoutField());

    // if (Robot.isSimulation()) {
    //   vision.enableSimulation(() -> RobotConfig.drive.getPose(), false);
    // }

    AutoNamedCommands.configure();
    autoChooser = AutoBuilder.buildAutoChooser("None");
  }
}
