package frc.robot.subsystems.interfaces;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Arm extends Subsystem {
  public static class Constants {
    public static double defaultSpeedInVolts = 1.0;

    public static double maxAngleInDegrees = 90.0;
    public static double minAngleInDegrees = 0.0;

    public static double maxVelocityInDegreesPerSecond = 45;
    public static double maxAccelerationInDegreesPerSecondSquared = 120;

    public static double pidKp = 0.1;
    public static double pidKi = 0.0;
    public static double pidKd = 0.0;
    public static double pidTimeoutInSeconds = 3.0;

    public static double ffKs = 0.0;
    public static double ffKv = 0.0;
    public static double ffKa = 0.0;
    public static double ffKg = 0.1;
  }

  // gets the angle of the arm
  public double getAngle();

  public double getTargetAngle();

  public boolean isAtMaxLimit();

  public boolean isAtMinLimit();

  public double getVelocity();

  // sets of the angle of the arm
  public void setAngle(double degrees);

  public boolean isAtSetpoint();

  public Command getStowCommand();

  public default void add2dSim(Mechanism2d mech2d) {}
}
