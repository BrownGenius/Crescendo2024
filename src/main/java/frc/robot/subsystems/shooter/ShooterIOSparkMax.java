package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;

public class ShooterIOSparkMax implements ShooterIO {
  // Gear ratio for the shooter mechanism
  private static final double GEAR_RATIO = 1.0;

  // define the 1 SparkMax Controllers
  private final SparkMax flywheel;

  // Gets the NEO encoder
  private final RelativeEncoder encoder;

  SparkMaxConfig flyWheelConfig = new SparkMaxConfig();

  public ShooterIOSparkMax(int id) {
    flywheel = new SparkMax(id, MotorType.kBrushless);
    // // Set motor to brake mode so shooter stops spinning immediately

    // // Last thing we do is save all settings to flash on sparkmax

    flyWheelConfig
        .inverted(false)
        .smartCurrentLimit(60, 40)
        .secondaryCurrentLimit(20000)
        .idleMode(SparkBaseConfig.IdleMode.kBrake);

    flywheel.configure(
        flyWheelConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    encoder = flywheel.getEncoder();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Set velocityRadPerSec to the encoder velocity(rotationsPerMinute) divided by the gear ratio
    // and converted into Radians Per Second
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    // Get applied voltage from the top motor
    inputs.appliedVolts = flywheel.getAppliedOutput() * flywheel.getBusVoltage();
    // Get applied voltage from the top motor
    inputs.current = flywheel.getOutputCurrent();
  }

  public boolean supportsHardwarePid() {
    return false;
  }

  @Override
  public void setVoltage(double volts) {
    // Set the voltage output for the top motor
    flywheel.setVoltage(volts);
  }
}
