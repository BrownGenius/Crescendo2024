package frc.robot.commands.debug;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.shooter.SetShooterVelocity;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.function.DoubleSupplier;

public class PrepareForScore extends ParallelCommandGroup {
  public PrepareForScore(
      Arm arm, Shooter shooter, DoubleSupplier armAngle, DoubleSupplier shooterVelocity) {
    addCommands(
        new ArmToPosition(arm, armAngle).withTimeout(ArmSubsystem.Constants.pidTimeoutInSeconds));
    addCommands(
        new SetShooterVelocity(shooter, shooterVelocity)
            .withTimeout(ShooterSubsystem.Constants.pidTimeoutInSeconds));
  }
}
