package frc.robot.commands.game.crescendo2024.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.common.arm.ArmToPosition;
import frc.robot.commands.common.shooter.SetShooterVelocity;
import frc.robot.subsystems.interfaces.Arm;
import frc.robot.subsystems.interfaces.Shooter;
import java.util.function.DoubleSupplier;

public class AutoPrepareForScore extends SequentialCommandGroup {
  DoubleSupplier armAngleInDegrees;
  DoubleSupplier shooterVelocityInRPMs;

  public AutoPrepareForScore(
      Arm arm,
      Shooter shooter,
      DoubleSupplier armAngleInDegrees,
      DoubleSupplier shooterVelocityInRPMs) {
    if (Constants.debugCommands) {
      addCommands(
          new PrintCommand(
                  "START: AutoPrepareToScore"
                      + " angle: "
                      + armAngleInDegrees.getAsDouble()
                      + " velocity: "
                      + shooterVelocityInRPMs.getAsDouble())
              .onlyIf(() -> Constants.debugCommands));
    }
    addCommands(
        new ParallelCommandGroup(
            new ArmToPosition(arm, armAngleInDegrees)
                .withTimeout(Arm.Constants.pidTimeoutInSeconds),
            new SetShooterVelocity(shooter, shooterVelocityInRPMs)
                .withTimeout(Shooter.Constants.pidTimeoutInSeconds)));

    if (Constants.debugCommands) {
      addCommands(new PrintCommand("  END: AutoPrepareToScore"));
    }
  }
}
