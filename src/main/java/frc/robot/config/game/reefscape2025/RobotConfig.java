package frc.robot.config.game.reefscape2025;

import frc.robot.config.common.RobotConfigBase;
import frc.robot.io.implementations.arm.ArmIOStub;
import frc.robot.subsystems.implementations.arm.ArmSubsystem;

/* Put all constants here with reasonable defaults */
public class RobotConfig extends RobotConfigBase {
  public static ArmSubsystem arm;

  public RobotConfig(boolean stubDrive, boolean stubAuto, boolean stubVision, boolean stubArm) {
    super(stubDrive, stubAuto, stubVision);

    if (stubArm) {
      arm = new ArmSubsystem(new ArmIOStub());
    }
  }
}
