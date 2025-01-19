package frc.robot.commands.game.crescendo2024.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.common.arm.ArmToPosition;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.intake.Intake;
import java.util.function.DoubleSupplier;

public class AutoPrepareForIntake extends SequentialCommandGroup {
  public AutoPrepareForIntake(
      Arm arm, Intake intake, DoubleSupplier intakeAngle, DoubleSupplier intakeVoltage) {
    if (Constants.debugCommands) {
      addCommands(new PrintCommand("START: " + this.getClass().getSimpleName()));
    }
    if (intakeVoltage == null) {
      addCommands(
          new ParallelCommandGroup(new ArmToPosition(arm, intakeAngle)),
          new AutoIndexPiece(intake));
    } else {
      addCommands(
          new ParallelCommandGroup(new ArmToPosition(arm, intakeAngle)),
          new AutoIndexPiece(intake, intakeVoltage));
    }
    if (Constants.debugCommands) {
      addCommands(new PrintCommand("  END: " + this.getClass().getSimpleName()));
    }
  }

  // Sets the Arm to the ideal angle for intake
  // Turns the intake motor on
  public AutoPrepareForIntake(Arm arm, Intake intake) {
    this(arm, intake, () -> ArmSubsystem.Constants.intakeAngleInDegrees, null);
  }
}
