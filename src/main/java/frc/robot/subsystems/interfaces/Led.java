package frc.robot.subsystems.interfaces;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;

public interface Led {
  public static class Constants {
    public static int Led1PWDPort = 0;
    public static int Led1Length = 34;

    public static int Led2PWDPort = 1;
    public static int Led2Length = 60;
  }

  public default void setColor(int red, int green, int bluw) {}

  public default int getRed() {
    return 0;
  }

  public default int getGreen() {
    return 0;
  }

  public default int getBlue() {
    return 0;
  }

  public default void add2dSim(Mechanism2d mech2d) {}

  public Command getNoteDetectionCommand();

  public Command getAmpModeCommand();

  public Command getSpeakerModeCommand();
}
