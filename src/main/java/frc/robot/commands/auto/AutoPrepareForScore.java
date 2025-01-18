package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.shooter.SetShooterVelocity;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSubsystem;
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
                .withTimeout(ArmSubsystem.Constants.pidTimeoutInSeconds),
            new SetShooterVelocity(shooter, shooterVelocityInRPMs)
                .withTimeout(ShooterSubsystem.Constants.pidTimeoutInSeconds)));

    if (Constants.debugCommands) {
      addCommands(new PrintCommand("  END: AutoPrepareToScore"));
    }
  }
}
