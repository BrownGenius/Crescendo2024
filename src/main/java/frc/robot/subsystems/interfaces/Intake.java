package frc.robot.subsystems.interfaces;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;

public interface Intake {

  public static class Constants {
    public static double defaultSpeedInVolts = 6.0;
  }

  public default boolean isPieceDetected() {
    return false;
  }

  public default void runVoltage(double volts) {}

  public default double getCurrentVoltage() {
    return 0;
  }

  public default void turnOff() {
    runVoltage(0);
  }

  public Command getTurnOffCommand();

  public default void turnOn() {
    runVoltage(Constants.defaultSpeedInVolts);
  }

  public Command getTurnOnCommand();

  public default void add2dSim(Mechanism2d mech2d) {}
  ;
}
