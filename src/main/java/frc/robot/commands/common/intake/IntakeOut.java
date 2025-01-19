package frc.robot.commands.common.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.interfaces.Intake;

public class IntakeOut extends Command {
  Intake intake;

  public IntakeOut(Intake intake) {
    this.intake = intake;
    addRequirements((Subsystem) intake);
  }

  @Override
  public void initialize() {
    intake.runVoltage(Intake.Constants.defaultSpeedInVolts * -1);
  }

  @Override
  public void end(boolean interrupted) {
    intake.runVoltage(0);
  }
}
