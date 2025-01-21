// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.game.reefscape2025.*;

public class RobotContainer {
  public final RobotConfig robotConfig;
  private static final String robotNameKey = "Robot Name";

  public RobotContainer() {
    String robotName = "UNKNOWN";

    Preferences.initString(robotNameKey, robotName);
    robotName = Preferences.getString(robotNameKey, robotName);
    System.out.println("Loading Settings for Robot Name = " + robotName);
    switch (robotName) {
      case "PHOENIX":
        robotConfig = new RobotConfigPhoenix();
        break;
      case "UNKNOWN":
      default:
        /* If running simulation, put the robot config you want here */
        // robotConfig = new RobotConfigInferno();
        robotConfig = new RobotConfigPhoenix();
        // robotConfig = new RobotConfigSherman();
        // robotConfig = new RobotConfigStub();
    }

    robotConfig.configureBindings();
  }

  public Command getAutonomousCommand() {
    return RobotConfig.autoChooser.getSelected();
  }
}
