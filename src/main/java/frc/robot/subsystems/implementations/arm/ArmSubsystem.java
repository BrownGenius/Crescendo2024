package frc.robot.subsystems.implementations.arm;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.io.interfaces.ArmIO;
import frc.robot.io.interfaces.ArmIOInputsAutoLogged;
import frc.robot.subsystems.interfaces.Arm;
import frc.robot.util.TrapezoidProfileSubsystem2876;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends TrapezoidProfileSubsystem2876 implements Arm {
  public static class Constants {
    public static double absolutePositionOffset = 0; /* 0-1 */
    public static double absoluteEncoderInversion = 1; /* 1 for none, -1 to invert */

    public static double pidMaxOutput = 0.4;
    public static double pidMinOutput = -0.4;
    public static double pidAngleErrorInDegrees = 2.0;
    public static double pidSettlingTimeInSeconds = 0.1;

    public static double intakeAngleInDegrees = 1;
    public static double ejectAngleInDegrees = 15;
    public static double ampScoreAngleInDegrees = 80;
    public static double subwooferScoreAngleInDegrees = 10;
    public static double subwooferScoreFromPodiumAngleInDegrees = 20;
    public static double noteScoreAngleInDegrees = 25;
    public static double stowIntakeAngleInDegrees = 15;
    public static double matchStartArmAngle = 90;

    public static double maxBacklashDegrees = 0.0;

    // Arm Angle Calculations
    // Polynomial: y = a*x^2 + b*x + c
    //   y = angle and x = distance
    public static double minDistanceInMeters = 0; // min distance we can aim at
    public static double maxDistanceInMeters = 3.0; // max distance we can aim at
    public static double Ax2 = 0;
    public static double Bx = (Arm.Constants.maxAngleInDegrees / 2) / maxDistanceInMeters;
    public static double C = 0;
  }

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private ArmFeedforward feedforward =
      new ArmFeedforward(
          Arm.Constants.ffKs, Arm.Constants.ffKg, Arm.Constants.ffKv, Arm.Constants.ffKa);
  private final SysIdRoutine sysId;
  private final double positionDegreeMax = Arm.Constants.maxAngleInDegrees;
  private final double positionDegreeMin = Arm.Constants.minAngleInDegrees;
  @AutoLogOutput private double targetVoltage;
  @AutoLogOutput private double targetDegrees;
  @AutoLogOutput private double targetRelativeDegrees;
  @AutoLogOutput private double goalSetpointDegrees;
  @AutoLogOutput private double currentSetpointDegrees;
  private double backlashCompensationDirection = 0;

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final double armAngle2dOffset = 0;
  private MechanismLigament2d arm2d = null;

  private double kG, kV, kA, kS;

  public ArmSubsystem(ArmIO io) {
    super(
        new TrapezoidProfile.Constraints(
            Arm.Constants.maxVelocityInDegreesPerSecond,
            Arm.Constants.maxAccelerationInDegreesPerSecondSquared));
    this.io = io;

    kG = Arm.Constants.ffKg;
    kV = Arm.Constants.ffKv;
    kA = Arm.Constants.ffKa;
    kS = Arm.Constants.ffKs;

    // Configure SysId based on the AdvantageKit example
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Arm/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVoltage(voltage.in(Volts)), null, this));

    io.setBrakeMode(true);
    disable();
  }

  @Override
  public void useState(TrapezoidProfile.State setpoint) {
    double ff = feedforward.calculate(setpoint.position, 0);

    // Use feedforward +  HW velocity PID (ignore SW PID)
    io.setPosition(setpoint.position, ff);
    currentSetpointDegrees = setpoint.position;

    Logger.recordOutput("Arm/setAngle/setpointDegrees", setpoint.position);
    Logger.recordOutput("Arm/setAngle/ffVolts", ff);

    // System.out.println("pos: " + setpoint.position);
    // System.out.println("vel: " + setpoint.velocity);
  }

  public double getRelativeAngle() {
    return inputs.relativePositionDegrees;
  }

  @Override
  public TrapezoidProfile.State getMeasurement() {
    return new TrapezoidProfile.State(getRelativeAngle(), getVelocity());
  }

  @Override
  public double getAngle() {
    return inputs.positionDegrees;
  }

  @Override
  public double getVelocity() {
    return inputs.velocityDegrees;
  }

  @Override
  public double getTargetAngle() {
    return targetDegrees;
  }

  // sets of the angle of the arm
  @Override
  public void setAngle(double degrees) {
    degrees =
        MathUtil.clamp(degrees, Arm.Constants.minAngleInDegrees, Arm.Constants.maxAngleInDegrees);

    Logger.recordOutput("Arm/setAngle/requestedAngleDegress", degrees);
    // Don't try to set position if absolute encoder is broken/missing.
    if (isAbsoluteEncoderConnected() == false) {
      return;
    }
    if (isAbsoluteEncoderReadingValid() == false) {
      return;
    }

    // Clamp the target degrees
    this.targetDegrees = degrees;

    // We instantiate a new object here each time because constants can change when being tuned.
    feedforward = new ArmFeedforward(kS, kG, kV, kA);

    this.targetRelativeDegrees = this.targetDegrees;

    // If we are moving up, we need to account for backlash since the arm tends to bias down (due to
    // gravity)
    // if ((this.targetRelativeDegrees > inputs.relativePositionDegrees)) {
    //   backlashCompensationDirection = 1;
    // } else {
    //   backlashCompensationDirection = 0;
    // }
    this.targetRelativeDegrees +=
        (backlashCompensationDirection * ArmSubsystem.Constants.maxBacklashDegrees);

    Logger.recordOutput("Arm/setAngle/setpointDegrees", this.targetRelativeDegrees);
    this.goalSetpointDegrees = this.targetRelativeDegrees;

    setGoal(this.goalSetpointDegrees);
    enable();
  }

  public boolean isAbsoluteEncoderConnected() {
    return io.isAbsoluteEncoderConnected();
  }

  public boolean isAbsoluteEncoderReadingValid() {
    if (getAngle() > Arm.Constants.minAngleInDegrees - 10
        && getAngle() < Arm.Constants.maxAngleInDegrees + 10) {
      return true;
    }
    return false;
  }

  // Sets the voltage to volts. the volts value is -12 to 12
  public void runVoltage(double volts) {
    targetVoltage = voltageSafety(volts);
    disable();
    io.setVoltage(targetVoltage);
  }

  protected double voltageSafety(double voltage) {
    if (isLimitReached(voltage)) return 0.0;
    else return voltage;
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  @Override
  public void periodic() {
    super.periodic();
    // Updates the inputs
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    /*
    if (DevilBotState.getState() == State.DISABLED && io.isAbsoluteEncoderConnected()) {
      io.resetRelativeEncoder(getAngle());
    }
    */

    if (isLimitHigh() && inputs.appliedVolts > 0) {
      io.setVoltage(0);
    }
    if (isLimitLow() && inputs.appliedVolts < 0) {
      io.setVoltage(0);
    }

    if (null != arm2d) {
      arm2d.setAngle(inputs.positionDegrees + armAngle2dOffset);
    }
  }

  private boolean isLimitHigh() {
    // if abs encoder is broken/missing then we cannot detect limits, assume we are at limit.
    if (isAbsoluteEncoderConnected() == false) {
      return true;
    }
    if (inputs.positionDegrees >= positionDegreeMax) {
      inputs.limitHigh = true;
    } else {
      inputs.limitHigh = false;
    }
    return inputs.limitHigh;
  }

  private boolean isLimitLow() {
    // if abs encoder is broken/missing then we cannot detect limits, assume we are at limit.
    if (isAbsoluteEncoderConnected() == false) {
      return true;
    }
    if (inputs.positionDegrees <= positionDegreeMin) {
      inputs.limitLow = true;

    } else {
      inputs.limitLow = false;
    }
    return inputs.limitLow;
  }

  private boolean isLimitReached(double desiredVoltage) {
    if (isLimitHigh() && desiredVoltage > 0.0) {
      return true;
    }
    if (isLimitLow() && desiredVoltage < 0.0) {
      return true;
    }
    return false;
  }

  @Override
  public boolean isAtMaxLimit() {
    return isLimitHigh();
  }

  @Override
  public boolean isAtMinLimit() {
    return isLimitLow();
  }

  private void stow() {
    setAngle(ArmSubsystem.Constants.stowIntakeAngleInDegrees);
  }

  @Override
  public Command getStowCommand() {
    return runOnce(() -> stow());
  }

  @Override
  public void add2dSim(Mechanism2d mech2d) {
    MechanismRoot2d armPivot2d = mech2d.getRoot("Arm Pivot", 15, 10);
    armPivot2d.append(new MechanismLigament2d("Arm Tower", 10, -90));
    arm2d =
        armPivot2d.append(
            new MechanismLigament2d(
                "Arm",
                30,
                inputs.positionDegrees + armAngle2dOffset,
                6,
                new Color8Bit(Color.kYellow)));
  }

  @Override
  public boolean isAtSetpoint() {
    return (Math.abs(currentSetpointDegrees - goalSetpointDegrees)
        < ArmSubsystem.Constants.pidAngleErrorInDegrees);
  }
}
