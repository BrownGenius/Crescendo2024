package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double absolutePositionRad = 0.0;
    public double absolutePositionDegree = 0.0;
    public double absolutePositionRaw;
    public boolean absoluteEncoderConnected = false;
    public double current;
    public double appliedVolts = 0.0;
    public double relativePositionRotations = 0.0;
    public boolean limitHigh = false;
    public boolean limitLow = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  /** Run the arm motor at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** sets of the position of the arm */
  public default void setPosition(double degree, double ffVolts) {}

  /** Reset relative encoder position to given */
  public default void resetRelativeEncoder(double position) {}

  /** Is absolute encoder connected */
  public default boolean isAbsoluteEncoderConnected() {
    return true;
  }

  /** Set PID constants */
  public default void setFeedback(
      double kP, double kI, double kD, double minOuput, double maxOutput) {}

  /** Set brake mode on motors */
  public default void setBrakeMode(boolean brake) {}
}
