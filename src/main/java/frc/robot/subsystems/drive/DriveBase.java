package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBase extends SubsystemBase implements Drive {
  public static class Constants {
    public static double rotatePidErrorInDegrees = 2.0;
    public static double pidTimeoutInSeconds = 0.5;
    public static double pidSettlingTimeInSeconds = 0.1;

    public static double slewRateLimiterX = 3;
    public static double slewRateLimiterY = 3;
    public static double slewRateLimiterAngle = 3;

    public static double blueSpeakerX = 0.14;
    public static double speakerY = 5.53;
    public static double redSpeakerX = 16.54 - 0.14;
  }

  @Override
  public void runVelocity(ChassisSpeeds velocity) {}

  @Override
  public double getMaxLinearSpeed() {
    return 0;
  }

  @Override
  public double getMaxAngularSpeed() {
    return 0;
  }
}
