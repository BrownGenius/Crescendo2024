package frc.robot.commands.game.crescendo2024.debug;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.common.arm.ArmToPosition;
import frc.robot.commands.common.shooter.SetShooterVelocity;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

public class PrepareForScore extends ParallelCommandGroup {
  public PrepareForScore(
      Arm arm, Shooter shooter, DoubleSupplier armAngle, DoubleSupplier shooterVelocity) {
    addCommands(new ArmToPosition(arm, armAngle).withTimeout(Arm.Constants.pidTimeoutInSeconds));
    addCommands(
        new SetShooterVelocity(shooter, shooterVelocity)
            .withTimeout(Shooter.Constants.pidTimeoutInSeconds));
  }
}
