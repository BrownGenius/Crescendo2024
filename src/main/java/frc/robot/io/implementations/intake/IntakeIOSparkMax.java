package frc.robot.io.implementations.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.io.interfaces.IntakeIO;

public class IntakeIOSparkMax implements IntakeIO {
  private static final double GEAR_RATIO = 4.0;

  // define the 1 SparkMax Controller
  private final SparkMax leader;
  // Gets the NEO encoder
  private final RelativeEncoder encoder;
  DigitalInput limitSwitchIntake = new DigitalInput(1);
  DigitalInput limitSwitchIntakeSecondary = new DigitalInput(2);

  SparkMaxConfig leaderConfig = new SparkMaxConfig();

  public IntakeIOSparkMax(int id, boolean inverted) {
    leader = new SparkMax(id, MotorType.kBrushless);
    encoder = leader.getEncoder();

    // leader motor is not inverted, and set follower motor to follow the leader
    leaderConfig.inverted(inverted).idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(35);
    leader.configure(
        leaderConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.limitSwitchIntake =
        !limitSwitchIntake.get()
            || !limitSwitchIntakeSecondary
                .get(); // Assume note in intake if either sensor is triggered
    inputs.current = leader.getOutputCurrent();
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
  }

  @Override
  public void setVoltage(double volts) {
    // Set the voltage output for the leader motor
    leader.setVoltage(volts);
  }
}
