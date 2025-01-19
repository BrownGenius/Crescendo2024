package frc.robot.io.implementations.intake;

import frc.robot.io.interfaces.IntakeIO;
import frc.robot.subsystems.implementations.intake.IntakeSubsystem;

public class IntakeIOStub implements IntakeIO {
  // private FlywheelSim sim = new FlywheelSim(DCMotor.getNEO(1), 1.5, 0.004);
  private double appliedVolts = 0.0;
  private double elapsedTimeMotorOn = 0.0;
  private boolean limitSwitchState = true; // Start with piece inserted

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    // sim.update(0.02);

    // inputs.positionRad = 0.0;
    // inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.limitSwitchIntake = limitSwitchState;
    // inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};

    // Simulate intake/shooter limit switches
    if (appliedVolts != 0.0) {
      elapsedTimeMotorOn += 0.02;

      if ((elapsedTimeMotorOn > IntakeSubsystem.Constants.intakeTimeoutInSeconds / 2)) {
        limitSwitchState = !limitSwitchState;
        elapsedTimeMotorOn = 0.0;
      }
    } else {
      elapsedTimeMotorOn = 0.0;
    }
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    // sim.setInputVoltage(volts);
  }
}
