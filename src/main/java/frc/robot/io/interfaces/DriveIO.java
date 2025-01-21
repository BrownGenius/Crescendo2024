package frc.robot.io.interfaces;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;
import swervelib.SwerveDrive;

public class DriveIO {
  @AutoLog
  public static class DriveIOInputs {
    public Pose2d pose;
    public double poseX = 0.0;
    public double poseY = 0.0;
    public double poseRotInDegrees = 0.0;
    public double distanceFromSpeaker = 0;
  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(DriveIOInputs inputs, SwerveDrive swerveDrive) {
    inputs.pose = swerveDrive.getPose();
    inputs.poseX = inputs.pose.getTranslation().getX();
    inputs.poseY = inputs.pose.getTranslation().getY();
    inputs.poseRotInDegrees = inputs.pose.getRotation().getDegrees();
    /*
        if (!DevilBotState.isRedAlliance()) {
          inputs.distanceFromSpeaker =
              Math.sqrt(
                  Math.pow(inputs.poseX - DriveBase.Constants.blueSpeakerX, 2)
                      + Math.pow(inputs.poseY - DriveBase.Constants.speakerY, 2));
        } else {
          inputs.distanceFromSpeaker =
              Math.sqrt(
                  Math.pow(inputs.poseX - DriveBase.Constants.redSpeakerX, 2)
                      + Math.pow(inputs.poseY - DriveBase.Constants.speakerY, 2));
        }
        ;
    */
  }
  // Other methods for controlling the drive subsystem...
}
