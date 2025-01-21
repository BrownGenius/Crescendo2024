package frc.robot.commands.common.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.Drive;
import frc.robot.util.DevilBotState;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommand extends Command {
  Drive drive;
  DoubleSupplier speedX;
  DoubleSupplier speedY;
  DoubleSupplier rot;
  Supplier<Optional<Double>> autoRot;
  double xSpeed;
  double ySpeed;
  double newRot;
  PIDController turnPID;

  public DriveCommand(
      Drive drive,
      DoubleSupplier speedX,
      DoubleSupplier speedY,
      DoubleSupplier rot,
      Supplier<Optional<Double>> autoRot) {
    this.drive = drive;
    this.speedX = speedX;
    this.speedY = speedY;
    this.rot = rot;
    this.autoRot = autoRot;

    // Wild guess at P constant.
    turnPID =
        new PIDController(
            Drive.Constants.rotatePidKp, Drive.Constants.rotatePidKi, Drive.Constants.rotatePidKd);
    turnPID.enableContinuousInput(0, 360);

    addRequirements((SubsystemBase) drive);
  }

  public DriveCommand(
      Drive drive, DoubleSupplier speedX, DoubleSupplier speedY, DoubleSupplier rot) {
    this(drive, speedX, speedY, rot, () -> Optional.empty());
  }

  @Override
  public void execute() {
    xSpeed = speedX.getAsDouble();
    ySpeed = speedY.getAsDouble();
    newRot = rot.getAsDouble();

    //    case "Squared Mode":
    if (xSpeed < 0) {
      xSpeed = Math.pow(xSpeed, 2) * -1;

    } else {
      xSpeed = Math.pow(xSpeed, 2);
    }
    if (ySpeed < 0) {
      ySpeed = Math.pow(ySpeed, 2) * -1;
    } else {
      ySpeed = Math.pow(ySpeed, 2);
    }
    if (newRot < 0) {
      newRot = Math.pow(newRot, 2) * -1;
    } else {
      newRot = Math.pow(newRot, 2);
    }

    if (this.autoRot.get().isPresent()) {
      double currentAngle = drive.getAngle();
      newRot = turnPID.calculate(currentAngle, currentAngle - this.autoRot.get().get());
    }

    /* Invert strafe controls if Field Oriented and on the Red Alliance */
    if (drive.isFieldOrientedDrive() && (DevilBotState.isRedAlliance())) {
      xSpeed *= -1;
      ySpeed *= -1;
    }

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            xSpeed * drive.getMaxLinearSpeed(),
            ySpeed * drive.getMaxLinearSpeed(),
            newRot * drive.getMaxAngularSpeed());

    drive.runVelocity(speeds);
  }
}
