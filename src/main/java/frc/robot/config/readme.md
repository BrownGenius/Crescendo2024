The config directory is used to support multiple robot versions using the same code base.

# Useful Resources:
* [Crescendo 2024 Main Electrical Components](https://docs.google.com/spreadsheets/d/1jhis3_a5TAV7oP3p6C41bP5yMevQPRnBtGeEiesHzKM)
* [Crescendo 2024 Hardware/Software Calibration](https://docs.google.com/document/d/1msJO2dKCxqzbMlSSOtfs8W_ITjltEbtXaQBxc-CpNMo)


# 2024
We had 3 different robots with different capabilities for dev/testing purposes:

1. Sherman: Tank Drive + Initial Arm Prototype
2. Phoenix: Initial Swerve Drive + Vision Prototype
3. Inferno: Final Robot

A [RobotConfig](RobotConfig.java) class defines all of the subsystems and corresponding constants required for a "robot". Each robot variant extends RobotConfig and instantiates the subsystems and overrides constants as needed.

| Robot | [Drive](../subsystems/drive/) | [Shooter](../subsystems/shooter/) | [Intake](../subsystems/intake/) | [Arm](../subsystems/arm/) | [Auto](../../../../deploy/pathplanner/) | [Climber](../subsystems/climber/) | [Vision](../subsystems/vision/) | [LED](../subsystems/led/) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| [Sherman](RobotConfigSherman.java) | Differential | SparkMax(2) | SparkMax(1) | TalonSRX(3) | No (stub) | No (stub) | No (stub) | No (stub) | No (stub) |
| [Phoenix](RobotConfigPhoenix.java) | Swerve(yagsl) | No (stub) | No (stub) | No (stub) | Yes | No (stub) | Shooter | No (stub) |
| [Inferno](RobotConfigInferno.java) | Swerve(yagsl) | SparkMax(2) | SparkMax(3) | SparkMax(4) | Yes | SparkMax(7) SparkMax(6) | Shooter Intake Right Left | Yes |
