package frc.robot.subsystems.arm;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.ArmConstants;

public class ArmIOSparkMax implements ArmIO {

  private final SparkMax motor;
  private final RelativeEncoder relEncoder;
  private final DutyCycleEncoder absoluteEncoder;
  private final SparkClosedLoopController armPid;
  public double lkP, lkI, lkD, lkIz, lkFF, lkMaxOutput, lkMinOutput, lmaxRPS;

  SparkMaxConfig config = new SparkMaxConfig();

  public ArmIOSparkMax(int id) {
    this(id, false);
  }

  double getOffsetCorrectedAbsolutePositionInRadians() {
    return ((absoluteEncoder.get() - ArmConstants.absolutePositionOffset)
            * ArmConstants.absoluteEncoderInversion)
        * 2.0
        * Math.PI;
  }

  public ArmIOSparkMax(int id, boolean inverted) {
    /* Instantiate 1x SparkMax motors and absolute encoder */
    motor = new SparkMax(id, MotorType.kBrushless);

    // first thing we do to spark device is reset it to known defaults.

    relEncoder = motor.getEncoder();
    relEncoder.setPosition(Units.radiansToDegrees(getOffsetCorrectedAbsolutePositionInRadians()));
    armPid = motor.getClosedLoopController();
    absoluteEncoder = new DutyCycleEncoder(0);

    // This will need to be set from a constant once we have the arm assembled and can measure the
    // offset.  Once the arm is done this value won't change.  It can change if arm chain slips so
    // check it after any mechanican work is done.  Also the decimal places matter.  Don't round or
    // leave off numbers.
    //
    System.out.println("ArmIOSparkMax(): Absolute Position Offset: " + absoluteEncoder.get());
    absoluteEncoder.setDutyCycleRange(1.0 / 1024.0, 1023.0 / 1024.0);

    config.inverted(inverted).smartCurrentLimit(40);

    // Initialize the relative encoder position based on absolute encoder position.  The abs and rel
    // encoder do not scale/align 1-1. At zero they are both zero.  When rel encoder is 80, abs
    // encoder is not. Perhaps we just say that if abs encoder is at 80, set rel to X, or abs
    // encoder is at 0, set rel to 0.  Everything else is invalid and requires arm to rehome itself.
    //
    // relEncoder.setPosition(0);

    // 60:1 gear box, 72 teeth on the arm cog and 14 teeth on the motor cog
    double gearRatio = (60.0 * (72.0 / 14.0));
    double rotationsToDegreesConversionFactor = 360.0 / gearRatio;

    config.encoder.positionConversionFactor(rotationsToDegreesConversionFactor);
    config.encoder.positionConversionFactor(rotationsToDegreesConversionFactor / 60.0);

    lkP = RobotConfig.ArmConstants.pidKp;
    lkI = RobotConfig.ArmConstants.pidKi;
    lkD = RobotConfig.ArmConstants.pidKd;
    lkIz = 0;
    lkFF = 0;
    lkMaxOutput = RobotConfig.ArmConstants.pidMaxOutput;
    lkMinOutput = RobotConfig.ArmConstants.pidMinOutput;
    lmaxRPS = 300;

    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(lkP, lkI, lkD)
        .iZone(lkIz)
        .velocityFF(lkFF)
        .outputRange(lkMinOutput, lkMaxOutput);

    motor.configure(
        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    // SmartDashboard.putNumber("Arm/pid/P Gain", lkP);
    // SmartDashboard.putNumber("Arm/pid/I Gain", lkI);
    // SmartDashboard.putNumber("Arm/pid/D Gain", lkD);
    // SmartDashboard.putNumber("Arm/pid/I Zone", lkIz);
    // SmartDashboard.putNumber("Arm/pid/Feed Forward", lkFF);
    // SmartDashboard.putNumber("Arm/pid/Max Output", lkMaxOutput);
    // SmartDashboard.putNumber("Arm/pid/Min Output", lkMinOutput);

    // Last thing we do is burn config to spark flash
    // motor.burnFlash();
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.positionDegrees = Units.radiansToDegrees(getOffsetCorrectedAbsolutePositionInRadians());
    inputs.velocityDegrees = relEncoder.getVelocity();

    inputs.absoluteEncoderConnected = isAbsoluteEncoderConnected();
    inputs.absolutePositionRaw = absoluteEncoder.get();
    inputs.relativePositionDegrees = relEncoder.getPosition();
    inputs.positionError = inputs.positionDegrees - inputs.relativePositionDegrees;

    // Code below allows PID to be tuned using SmartDashboard.  And outputs extra data to
    // SmartDashboard.
    if (Constants.debugMode) {
      //   double lp = SmartDashboard.getNumber("Arm/pid/P Gain", 0);
      //   double li = SmartDashboard.getNumber("Arm/pid/I Gain", 0);
      //   double ld = SmartDashboard.getNumber("Arm/pid/D Gain", 0);
      //   double liz = SmartDashboard.getNumber("Arm/pid/I Zone", 0);
      //   double lff = SmartDashboard.getNumber("Arm/pid/Feed Forward", 0);
      //   double lmax = SmartDashboard.getNumber("Arm/pid/Max Output", 0);
      //   double lmin = SmartDashboard.getNumber("Arm/pid/Min Output", 0);

      //   if ((lp != lkP)) {
      //     armPid.setP(lp);
      //     lkP = lp;
      //   }
      //   if ((li != lkI)) {
      //     armPid.setI(li);
      //     lkI = li;
      //   }
      //   if ((ld != lkD)) {
      //     armPid.setD(ld);
      //     lkD = ld;
      //   }
      //   if ((liz != lkIz)) {
      //     armPid.setIZone(liz);
      //     lkIz = liz;
      //   }
      //   if ((lff != lkFF)) {
      //     armPid.setFF(lff);
      //     lkFF = lff;
      //   }
      //   if ((lmax != lkMaxOutput) || (lmin != lkMinOutput)) {
      //     armPid.setOutputRange(lmin, lmax);
      //     lkMinOutput = lmin;
      //     lkMaxOutput = lmax;
      //   }

      SmartDashboard.putBoolean("Arm/absEncoder/connected", absoluteEncoder.isConnected());

      // Try rebooting the robot with the arm/encoder in different positions.  Do these value
      // change?
      // How? Record observations so you can share with rest of us
      SmartDashboard.putNumber("Arm/absEncoder/absolutePos", absoluteEncoder.get());

      // This should show what the relative encoder is reading.  When you use the sparkmax position
      // PID it expects a setpoint in rotations.  Not clear if that means degrees or what unit is
      // used.
      SmartDashboard.putNumber("Arm/relEncoder/getPosition", relEncoder.getPosition());
    }
  }

  @Override
  public void setPosition(double degrees, double ffVolts) {
    if (Constants.debugMode) {
      SmartDashboard.putNumber("Arm/setPosition/degrees", degrees);
      SmartDashboard.putNumber("Arm/setPosition/ffVolts", ffVolts);
    }
    armPid.setReference(
        degrees,
        SparkMax.ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  /** Run the arm motor at the specified voltage. */
  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setFeedback(double kP, double kI, double kD, double minOutput, double maxOutput) {
    config.closedLoop.pid(kP, kI, kD).outputRange(minOutput, maxOutput);
    motor.configure(
        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public boolean isAbsoluteEncoderConnected() {
    return absoluteEncoder.isConnected();
  }

  @Override
  public void resetRelativeEncoder(double position) {
    // System.out.println("resetRelativeEncoder " + position);
    relEncoder.setPosition(position);
  }

  @Override
  public void setBrakeMode(boolean brake) {
    SparkBaseConfig.IdleMode mode;
    if (brake) {
      mode = SparkBaseConfig.IdleMode.kBrake;
      if (Constants.debugMode) {
        SmartDashboard.putString("Arm/Idle Mode", "kBrake");
      }
    } else {
      mode = SparkBaseConfig.IdleMode.kCoast;
      if (Constants.debugMode) {
        SmartDashboard.putString("Arm/Idle Mode", "kCoast");
      }
    }
    config.idleMode(mode);
    REVLibError error =
        motor.configure(
            config,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);
    if (error != REVLibError.kOk) {
      if (Constants.debugMode) {
        SmartDashboard.putString("Arm/Idle Mode", "Error");
      }
    }
  }
}
