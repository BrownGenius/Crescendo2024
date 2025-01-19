package frc.robot.config;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.commands.game.crescendo2024.auto.AutoNamedCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOSparkMax;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOSparkMax;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.drive.DriveSwerveYAGSL;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.led.LedIOWS121b;
import frc.robot.subsystems.led.LedSystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionCamera;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.ArrayList;

/* Override Inferno specific constants here */
public class RobotConfigInferno extends RobotConfig {
  public RobotConfigInferno() {
    super(false, false, false, false, false, false, false);

    // Inferno has a Swerve drive train
    Drive.Constants.rotatePidKp = 0.02;
    Drive.Constants.rotatePidKi = 0.0;
    Drive.Constants.rotatePidKd = 0.0;
    DriveBase.Constants.rotatePidErrorInDegrees = 0.5;
    Drive.Constants.pidTimeoutInSeconds = 0.5;
    DriveBase.Constants.pidSettlingTimeInSeconds = 0.5;

    drive = new DriveSwerveYAGSL("yagsl/inferno");

    // Inferno has a TalonSRX based intake
    // IntakeConstants.defaultSpeedInVolts = 6.0;

    // Reading Intake v2.0
    // IntakeConstants.sensorDelayFalseToTrueInSeconds = 0.06;
    IntakeSubsystem.Constants.sensorDelayTrueToFalseInSeconds = 0.1;

    // WPI Intake v1.0
    IntakeSubsystem.Constants.sensorDelayFalseToTrueInSeconds = 0.0;
    // IntakeConstants.sensorDelayTrueToFalseInSeconds = 0.1;

    // intake = new IntakeSubsystem(new IntakeIOTalonSRX(3, true));

    Intake.Constants.defaultSpeedInVolts = 10.0; // SparkMax/NEO based voltage
    intake = new IntakeSubsystem(new IntakeIOSparkMax(3, false));

    // Inferno has a single SparkMax based shooter

    // Values from Nilesh's Shooter SysId Run @ BHS on Inferno 2024-03-18
    Shooter.Constants.ffKs =
        0.0; // SysId calculated -0.016149, but likely erroneous.  Will need to re-run sysid
    Shooter.Constants.ffKv = 0.021208;
    Shooter.Constants.ffKa = 0.0072313;

    Shooter.Constants.pidKp = 0.0047154;
    Shooter.Constants.pidKi = 0.0;
    Shooter.Constants.pidKd = 0.0;
    ShooterSubsystem.Constants.pidVelocityErrorInRPM = 500;

    Shooter.Constants.maxVelocityInRPM = 6000;
    Shooter.Constants.maxAccelerationInRPMSquared = Shooter.Constants.maxVelocityInRPM * 4;
    ShooterSubsystem.Constants.ampScoreVelocityInRPM = 2500;
    ShooterSubsystem.Constants.velocityInRPM = 4500;
    shooter = new ShooterSubsystem(new ShooterIOSparkMax(2));

    ArmSubsystem.Constants.absolutePositionOffset =
        0.5031059375776484; // Determined empirically on 2024-03-15 (after replacing broken abs)
    // encoder)
    ArmSubsystem.Constants.absoluteEncoderInversion = -1;

    Arm.Constants.pidKp = 0.1;
    Arm.Constants.pidKi = 0.0;
    Arm.Constants.pidKd = 0.0;
    Arm.Constants.ffKs = 0.0;
    Arm.Constants.ffKg = 0.72;
    Arm.Constants.ffKv = 6.18;
    Arm.Constants.ffKa = 0.04;

    Arm.Constants.maxVelocityInDegreesPerSecond = 120;
    Arm.Constants.maxAccelerationInDegreesPerSecondSquared = 120;

    ArmSubsystem.Constants.pidMaxOutput = 6.0;
    ArmSubsystem.Constants.pidMinOutput = -5.0;

    ArmSubsystem.Constants.pidAngleErrorInDegrees = 1.5;
    Arm.Constants.maxAngleInDegrees = 89.0;
    Arm.Constants.minAngleInDegrees = -1.5;

    ArmSubsystem.Constants.intakeAngleInDegrees = -1.5;
    ArmSubsystem.Constants.ampScoreAngleInDegrees = 89.0;
    ArmSubsystem.Constants.subwooferScoreAngleInDegrees = 9.80;
    ArmSubsystem.Constants.subwooferScoreFromPodiumAngleInDegrees = 33; // min/max = 33.24/34.37
    ArmSubsystem.Constants.noteScoreAngleInDegrees =
        24.24; // Empirically at reading practice field after running 2-note center
    ArmSubsystem.Constants.stowIntakeAngleInDegrees = 5.0;
    ArmSubsystem.Constants.matchStartArmAngle = 90;
    Arm.Constants.pidTimeoutInSeconds = 2.0;

    ArmSubsystem.Constants.maxBacklashDegrees = 3.0;

    ArmSubsystem.Constants.minDistanceInMeters = Units.inchesToMeters(38);
    ArmSubsystem.Constants.maxDistanceInMeters = 4.0;
    ArmSubsystem.Constants.Ax2 = -3.2;
    ArmSubsystem.Constants.Bx = 23.7;
    ArmSubsystem.Constants.C = -10.3;
    arm = new ArmSubsystem(new ArmIOSparkMax(4, true));

    Climber.Constants.minPositionInRadians = 0.01;
    Climber.Constants.maxPositionInRadians = 27.875;
    Climber.Constants.defaultSpeedInVolts = 12.0;
    ClimberSubsystem.Constants.autoZeroVoltage = 2.0;
    ClimberSubsystem.Constants.autoZeroMaxCurrent = 16;
    ClimberSubsystem.Constants.autoZeroMinVelocity = 1.0;
    ClimberSubsystem.Constants.autoZeroExtendTimeInSeconds = 0.5;
    ClimberSubsystem.Constants.autoZeroMaxRetractTimeInSeconds =
        10.0 + ClimberSubsystem.Constants.autoZeroExtendTimeInSeconds;
    ClimberSubsystem.Constants.autoZeroOffset =
        -0.5; // When auto-zeroing, to reduce stress on the mechanism, this is the amount we want to
    ClimberSubsystem.Constants.matchStartPositionRadiansRight = 21;
    // retract the climber after auto-zeroing
    climber = new ClimberSubsystem(new ClimberIOSparkMax(7, false), new ClimberIOSparkMax(6, true));

    cameras = new ArrayList<VisionCamera>();
    /* TODO: Measure and set camera name/location */
    cameras.add(
        new VisionCamera(
            "shooter",
            "1188",
            new Transform3d(
                new Translation3d(-Units.inchesToMeters(9.5), 0, Units.inchesToMeters(14)),
                new Rotation3d(0, Units.degreesToRadians(-32), Units.degreesToRadians(180)))));

    cameras.add(
        new VisionCamera(
            "intake",
            "1184",
            new Transform3d(
                new Translation3d(0.3048, 0, 0.22),
                new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(0)))));

    cameras.add(
        new VisionCamera(
            "right",
            "1196",
            new Transform3d(
                new Translation3d(
                    -Units.inchesToMeters(5.25),
                    -Units.inchesToMeters(11.25),
                    Units.inchesToMeters(7)),
                new Rotation3d(0, Units.degreesToRadians(-32), Units.degreesToRadians(-90)))));

    cameras.add(
        new VisionCamera(
            "left",
            "1190",
            new Transform3d(
                new Translation3d(
                    -Units.inchesToMeters(5.25),
                    Units.inchesToMeters(11.25),
                    Units.inchesToMeters(7)),
                new Rotation3d(0, Units.degreesToRadians(-32), Units.degreesToRadians(90)))));

    VisionSubsystem.Constants.visionDistanceOffsetInMeters = -0.2;
    vision = new VisionSubsystem(cameras, AprilTagFields.k2024Crescendo.loadAprilTagLayoutField());

    if (Robot.isSimulation()) {
      vision.enableSimulation(() -> RobotConfig.drive.getPose(), false);
    }

    AutoNamedCommands.configure();
    autoChooser = AutoBuilder.buildAutoChooser("Sit Still");

    Led.Constants.Led1PWDPort = 9;
    Led.Constants.Led1Length = 34;
    led = new LedSystem(new LedIOWS121b());
  }
}
