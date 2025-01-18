package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.config.RobotConfig;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class AutoPrepareForIntakeV2 extends SequentialCommandGroup {
  public AutoPrepareForIntakeV2(Arm arm, Intake intake) {
    if (Constants.debugCommands) {
      addCommands(new PrintCommand("START: " + this.getClass().getSimpleName()));
    }
    addCommands(
        new ParallelCommandGroup(
                new InstantCommand(
                    () -> intake.runVoltage(Intake.Constants.defaultSpeedInVolts)), // run intake
                new ArmToPosition(
                    arm,
                    () ->
                        ArmSubsystem.Constants.intakeAngleInDegrees) // Move arm to intake position
                )
            .until(() -> intake.isPieceDetected())); // exit early if piece detected
    addCommands(
        new SequentialCommandGroup(
            new WaitUntilCommand(() -> RobotConfig.intake.isPieceDetected())
                .withTimeout(IntakeSubsystem.Constants.intakeTimeoutInSeconds)
                .handleInterrupt(
                    () ->
                        System.err.println(
                            "INTERRUPTED: "
                                + "Auto Wait For Piece")), // Wait for piece to be detected (with
            // timeout)
            new ParallelCommandGroup(
                new InstantCommand(() -> RobotConfig.intake.runVoltage(0)) // , // turn off intake
                )));

    if (Constants.debugCommands) {
      addCommands(new PrintCommand("  END: " + this.getClass().getSimpleName()));
    }
  }
}
