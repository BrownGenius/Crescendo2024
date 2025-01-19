package frc.robot.io.implementations.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import frc.robot.io.interfaces.ClimberIO;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;

public class ClimberIOSparkMax implements ClimberIO {
  private static final double GEAR_RATIO = 25; // make sure to change if nessessary

  // define the 2 SparkMax Controllers. A top, and a bottom
  private final SparkMax motor;

  private final RelativeEncoder encoder;

  SparkMaxConfig config = new SparkMaxConfig();

  public ClimberIOSparkMax(int id, boolean inverted) {
    motor = new SparkMax(id, MotorType.kBrushless);

    config
        .inverted(inverted)
        .smartCurrentLimit(30)
        .voltageCompensation(12.0)
        .idleMode(SparkBaseConfig.IdleMode.kBrake);

    motor.configure(
        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    encoder = motor.getEncoder();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    // Set velocityRadPerSec to the encoder velocity(rotationsPerMinute) divided by the gear ratio
    // and converted into Radians Per Second
    inputs.velocityRadiansPerSecond =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    // Get applied voltage from the top motor
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.current = motor.getOutputCurrent();
    inputs.positionRadians = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
  }

  @Override
  public void setVoltage(double volts) {
    // Set the voltage output for the top motor
    motor.setVoltage(volts);
    // bottom.setVoltage(volts);
  }

  @Override
  public void setPosition(double position) {
    encoder.setPosition(Units.radiansToRotations(position) * GEAR_RATIO);
  }
}
