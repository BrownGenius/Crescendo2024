package frc.robot.subsystems.implementations.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.Drive;

public class DriveBase extends SubsystemBase implements Drive {
  public static class Constants {
    public static double rotatePidErrorInDegrees = 2.0;
    public static double pidSettlingTimeInSeconds = 0.1;
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
