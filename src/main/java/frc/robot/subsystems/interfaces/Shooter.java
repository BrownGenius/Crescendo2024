package frc.robot.subsystems.interfaces;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;

public interface Shooter {
  public static class Constants {
    /* Feedforward */
    public static double ffKs = 0.088754;
    public static double ffKv = 0.029757;
    public static double ffKa = 0.01281;
    public static double ffKsBottom = 0.11831;
    public static double ffKvBottom = 0.029802;
    public static double ffKaBottom = 0.019246;

    /* PID */
    public static double pidKp = 0.043566;
    public static double pidKi = 0.0;
    public static double pidKd = 0.0;
    public static double pidTimeoutInSeconds = 2.0;

    public static double pidKpBottom = 0.04467;
    public static double pidKiBottom = 0.0;
    public static double pidKdBottom = 0.0;

    public static double defaultSpeedInVolts = 6.0;
    public static double maxVelocityInRPM = 4000;
    public static double maxAccelerationInRPMSquared = maxVelocityInRPM * 4;
  }

  /**
   * Set the voltage of motors for the shooter.
   *
   * @param volts The volts to set. Value should be between -12.0 and 12.0.
   */
  public default void runVoltage(double volts) {}

  /** Run closed loop at the specified velocity */
  public default void runVelocity(double velocityRPM) {}

  public boolean isAtSetpoint();

  public double getCurrentSpeed();

  public double getVoltage();

  public default void turnOff() {
    runVoltage(0);
  }

  public Command getTurnOffCommand();

  public default void add2dSim(Mechanism2d mech2d) {}
  ;
}
