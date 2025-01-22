package frc.robot.config.game.reefscape2025;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.subsystems.controls.drive.DriveControls;
import frc.robot.subsystems.implementations.drive.DriveBase;
import frc.robot.subsystems.implementations.vision.VisionCamera;
import frc.robot.subsystems.implementations.vision.VisionSubsystem;
import java.util.ArrayList;
import java.util.List;

/* Put all constants here with reasonable defaults */
public class RobotConfig {
  public static DriveBase drive;
  public static SendableChooser<Command> autoChooser;
  public static VisionSubsystem vision;
  public static List<VisionCamera> cameras;

  // Controls
  public CommandXboxController mainController = new CommandXboxController(0);
  private final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");

  // private final ShuffleboardTab pitTab = Shuffleboard.getTab("Pit");
  // private final ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");
  // private final ShuffleboardTab sysIdTestTab = Shuffleboard.getTab("SysId");

  public RobotConfig() {
    this(true, true, true);
  }

  public RobotConfig(boolean stubDrive, boolean stubAuto, boolean stubVision) {
    if (stubDrive) {
      drive = new DriveBase();
    }

    if (stubAuto) {
      autoChooser = new SendableChooser<>();
      autoChooser.setDefaultOption("No Auto Routines Specified", Commands.none());
    }

    if (stubVision) {
      cameras = new ArrayList<VisionCamera>();
      if (Robot.isSimulation()) {
        cameras.add(
            new VisionCamera(
                "photonvision",
                new Transform3d(
                    new Translation3d(-0.221, 0, .164),
                    new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(180)))));
        cameras.add(
            new VisionCamera(
                "left",
                new Transform3d(
                    new Translation3d(0, 0.221, .164),
                    new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(90)))));

        cameras.add(
            new VisionCamera(
                "right",
                new Transform3d(
                    new Translation3d(0, -0.221, .164),
                    new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(-90)))));
      }

      vision =
          new VisionSubsystem(
              cameras, AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape));

      if (Robot.isSimulation()) {
        vision.enableSimulation(() -> RobotConfig.drive.getPose(), false);
      }
    }
  }

  public void configureBindings() {
    // Send vision-based odometry measurements to drive's odometry calculations
    vision.setVisionMeasurementConsumer(drive::addVisionMeasurement);

    DriveControls.setupController(drive, mainController);
    DriveControls.addGUI(drive, driverTab);
  }
}
