package frc.robot.subsystems.controls.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.drive.DriveCommand;
import frc.robot.config.game.reefscape2025.RobotConfig;
import frc.robot.subsystems.interfaces.Drive;

public class DriveControls {
  public static void setupController(Drive drive, CommandXboxController controller) {
    SubsystemBase driveSubsystem = (SubsystemBase) drive;
    driveSubsystem.setDefaultCommand(
        new DriveCommand(
            drive,
            () -> MathUtil.applyDeadband(-controller.getLeftY(), 0.05), // Robot Strafe Front/Back
            () -> MathUtil.applyDeadband(-controller.getLeftX(), 0.05), // Robot Strafe Left/Right
            () -> MathUtil.applyDeadband(-controller.getRightX(), 0.05))); // Robot Rotate

    /* Debug/Test Only:
     *    Back Button = Zero Pose
     *    Start Button = Toggle Drive Orientation
     */
    controller.back().onTrue(new InstantCommand(() -> drive.resetOdometry()));
    controller
        .start()
        .onTrue(
            new InstantCommand(
                () -> {
                  if (drive.isFieldOrientedDrive()) {
                    drive.setFieldOrientedDrive(false);
                  } else {
                    drive.setFieldOrientedDrive(true);
                  }
                })); // Toggle Drive Orientation
  }

  public static void addGUI(Drive drive, ShuffleboardTab tab) {
    //    int colIndex = 0;
    //    int rowIndex = 0;

    /* Autonomous Chooser */
    tab.add("Autononmous", RobotConfig.autoChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        //        .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);

    /* Field Oriented Drive Indicator */
    tab.addBoolean("Field Oriented", () -> drive.isFieldOrientedDrive())
        .withWidget(BuiltInWidgets.kBooleanBox)
        //      .withPosition(colIndex, rowIndex++)
        .withSize(2, 1);
  }
}
