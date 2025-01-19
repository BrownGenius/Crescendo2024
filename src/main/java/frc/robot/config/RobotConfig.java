package frc.robot.config;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.io.implementations.arm.ArmIOStub;
import frc.robot.io.implementations.climber.ClimberIOStub;
import frc.robot.io.implementations.intake.IntakeIOStub;
import frc.robot.io.implementations.led.LedIOStub;
import frc.robot.io.implementations.shooter.ShooterIOStub;
import frc.robot.subsystems.implementations.arm.ArmSubsystem;
import frc.robot.subsystems.implementations.climber.ClimberSubsystem;
import frc.robot.subsystems.implementations.drive.DriveBase;
import frc.robot.subsystems.implementations.intake.IntakeSubsystem;
import frc.robot.subsystems.implementations.led.LedSystem;
import frc.robot.subsystems.implementations.shooter.ShooterSubsystem;
import frc.robot.subsystems.implementations.vision.VisionCamera;
import frc.robot.subsystems.implementations.vision.VisionSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/* Put all constants here with reasonable defaults */
public class RobotConfig {
  public static DriveBase drive;
  public static IntakeSubsystem intake;
  public static ShooterSubsystem shooter;
  public static ArmSubsystem arm;
  public static SendableChooser<Command> autoChooser;
  public static ClimberSubsystem climber;
  public static VisionSubsystem vision;
  public static RobotConfig instance;
  public static List<VisionCamera> cameras;
  public static LedSystem led;

  public Optional<Double> getArmAngleFromDistance(double distanceInMeters) {
    // Calculated using https://stats.blue/Stats_Suite/polynomial_regression_calculator.html
    // Based on empirical measurements done on 2024-04-03
    // (https://docs.google.com/spreadsheets/d/17Rh0MyVeME0KEAvkSqZKafnL0LPy9PoqWqOJ7ho49S8)

    distanceInMeters =
        MathUtil.clamp(
            distanceInMeters,
            ArmSubsystem.Constants.minDistanceInMeters,
            ArmSubsystem.Constants.maxDistanceInMeters);

    Optional<Double> angle =
        Optional.of(
            ArmSubsystem.Constants.Ax2 * Math.pow(distanceInMeters, 2)
                + ArmSubsystem.Constants.Bx * distanceInMeters
                + ArmSubsystem.Constants.C);

    // System.out.println("getArmAngleFromDistance(" + distanceInMeters + ") = " + angle.get());

    return angle;
  }

  public RobotConfig() {
    this(true, true, true);
  }

  public RobotConfig(boolean stubDrive, boolean stubShooter, boolean stubIntake) {
    this(stubDrive, stubShooter, stubIntake, true, true);
  }

  public RobotConfig(
      boolean stubDrive,
      boolean stubShooter,
      boolean stubIntake,
      boolean stubArm,
      boolean stubAuto) {
    this(stubDrive, stubShooter, stubIntake, stubArm, stubAuto, true);
  }

  public RobotConfig(
      boolean stubDrive,
      boolean stubShooter,
      boolean stubIntake,
      boolean stubArm,
      boolean stubAuto,
      boolean stubClimber) {
    this(stubDrive, stubShooter, stubIntake, stubArm, stubAuto, stubClimber, true);
  }

  public RobotConfig(
      boolean stubDrive,
      boolean stubShooter,
      boolean stubIntake,
      boolean stubArm,
      boolean stubAuto,
      boolean stubClimber,
      boolean stubVision) {
    this(stubDrive, stubShooter, stubIntake, stubArm, stubAuto, stubClimber, stubVision, true);
  }

  public RobotConfig(
      boolean stubDrive,
      boolean stubShooter,
      boolean stubIntake,
      boolean stubArm,
      boolean stubAuto,
      boolean stubClimber,
      boolean stubVision,
      boolean stubLed) {
    instance = this;

    if (stubDrive) {
      drive = new DriveBase();
    }

    if (stubShooter) {
      shooter =
          new ShooterSubsystem(
              new ShooterIOStub(ShooterIOStub.ShooterId.SHOOTER_TOP),
              new ShooterIOStub(ShooterIOStub.ShooterId.SHOOTER_BOTTOM));
    }

    if (stubIntake) {
      intake = new IntakeSubsystem(new IntakeIOStub());
    }

    if (stubArm) {
      arm = new ArmSubsystem(new ArmIOStub());
    }

    if (stubAuto) {
      autoChooser = new SendableChooser<>();
      autoChooser.setDefaultOption("No Auto Routines Specified", Commands.none());
    }

    if (stubClimber) {
      climber = new ClimberSubsystem(new ClimberIOStub(), new ClimberIOStub());
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
          new VisionSubsystem(cameras, AprilTagFields.k2024Crescendo.loadAprilTagLayoutField());

      if (Robot.isSimulation()) {
        vision.enableSimulation(() -> RobotConfig.drive.getPose(), false);
      }
    }
    if (stubLed) {
      led = new LedSystem(new LedIOStub());
    }
  }
}
