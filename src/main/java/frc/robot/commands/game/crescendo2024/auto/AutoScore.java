package frc.robot.commands.game.crescendo2024.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.common.arm.ArmToPosition;
import frc.robot.commands.common.drive.DriveToYaw;
import frc.robot.commands.common.shooter.SetShooterVelocity;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

public class AutoScore extends SequentialCommandGroup {
  DoubleSupplier robotYawInDegrees;
  DoubleSupplier armAngleInDegrees;
  DoubleSupplier shooterVelocityInRPMs;

  int isFinished = 0;
  int shotPieces = 0;

  public AutoScore(
      Drive drive,
      Arm arm,
      Intake intake,
      Shooter shooter,
      DoubleSupplier robotYawInDegrees,
      DoubleSupplier armAngleInDegrees,
      DoubleSupplier shooterVelocityInRPMs) {

    if (Constants.debugCommands) {
      addCommands(
          new PrintCommand(
                  "START: AutoScore yaw: "
                      + robotYawInDegrees.getAsDouble()
                      + " angle: "
                      + armAngleInDegrees.getAsDouble()
                      + " velocity: "
                      + shooterVelocityInRPMs.getAsDouble())
              .onlyIf(() -> Constants.debugCommands));
    }
    addCommands(
        new ParallelCommandGroup(
            new DriveToYaw(drive, robotYawInDegrees)
                .withTimeout(Drive.Constants.pidTimeoutInSeconds)
                .onlyIf(() -> robotYawInDegrees != null),
            new ArmToPosition(arm, armAngleInDegrees)
                .withTimeout(Arm.Constants.pidTimeoutInSeconds)
                .onlyIf(() -> armAngleInDegrees != null),
            new SetShooterVelocity(shooter, shooterVelocityInRPMs)
                .withTimeout(Shooter.Constants.pidTimeoutInSeconds)
                .onlyIf(() -> shooterVelocityInRPMs != null)));
    addCommands(new AutoScorePiece(intake, shooter));

    if (Constants.debugCommands) {
      addCommands(new PrintCommand("  END: AutoScore"));
    }
  }
}
