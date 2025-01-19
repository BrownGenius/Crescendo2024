package frc.robot.commands.game.crescendo2024.assist;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.common.arm.ArmToPosition;
import frc.robot.commands.common.intake.IntakeOut;
import frc.robot.subsystems.implementations.arm.ArmSubsystem;
import frc.robot.subsystems.interfaces.Arm;
import frc.robot.subsystems.interfaces.Intake;
import frc.robot.subsystems.interfaces.Shooter;

public class EjectPiece extends SequentialCommandGroup {

  // Raise arm if needed
  // Run intake in reverse
  // Turn off intake

  public EjectPiece(Intake intake, Arm arm, Shooter shooter) {

    double current_pos = arm.getAngle();
    if (current_pos < ArmSubsystem.Constants.ejectAngleInDegrees) {
      addCommands(new ArmToPosition(arm, () -> ArmSubsystem.Constants.ejectAngleInDegrees));
    }

    addCommands(
        new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new InstantCommand(() -> shooter.runVoltage(-12.0)),
                    new WaitCommand(0.50),
                    new InstantCommand(() -> shooter.runVoltage(0.0))),
                new IntakeOut(intake))
            .withTimeout(1));
  }
}
