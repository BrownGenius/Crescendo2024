// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ShooterEnable;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.subsystems.drive.DriveSwerveYAGSL;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RobotContainer {
  public final CommandXboxController controller;
  public final ShooterSubsystem shooter;
  public final DriveSwerveYAGSL drive;

  public RobotContainer() {
    controller = new CommandXboxController(0);

    shooter = new ShooterSubsystem(new ShooterIOSim());
    drive = new DriveSwerveYAGSL();

    configureBindings();
  }

  private void configureBindings() {
    shooter.setDefaultCommand(new InstantCommand(() -> shooter.disable(), shooter));

    controller.rightTrigger().whileTrue(new ShooterEnable(shooter));

    drive.setDefaultCommand(
        new DriveCommand(
            drive,
            () -> MathUtil.applyDeadband(-controller.getLeftY(), 0.01),
            () -> MathUtil.applyDeadband(-controller.getLeftX(), 0.01),
            () -> MathUtil.applyDeadband(-controller.getRightX(), 0.01)));
    // TODO: Move deadband to constants file

    controller
        .start()
        .onTrue(
            new InstantCommand(() -> drive.setFieldOrientedDrive(!drive.isFieldOrientedDrive())));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
