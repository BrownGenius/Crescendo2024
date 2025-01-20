Top-Level location of the robot's actual source code

This code based uses the [WPILib command-based](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html) programming paradigm

# Code Flow
1. [Main.java](Main.java) --> [Robot.java](Robot.java) --> [RobotContainer.java](RobotContainer.java)
   1. Setup Robot Config
      * A persistent preference with the key "Robot Name" is hard-coded onto the RoboRio flash memory
      * The "Robot Name" key is read from the RoboRio at start and determines which robot config to load.
   1. Setup Controls (Bindings)
   1. Setup Autonomous

# Directory Structure
* commands\ - all commands are stored here
   * common\ - Common generic commands that only use the subsystem *interfaces*. Can be re-used, as-is, year to year
   * game\
      * gameYYYY\ - E.g. "crescendo2024" - game and/or subsystem specific commands. Generally *cannot* be re-used, as-is, year to year
* config\ - all robot configurations are stored here
   * game\ - game specific robot configurations are stored here
      * gameYYYY\ E.g. "crescendo2024" - game specific robot configurations are stored here
* io\ - generic low-level hardware IO is contained here. Can be re-used, as-is, year to year.
   * interfaces\ - contains all low-level hard IO *interfaces* that abstract basic software contolled electronic components (e.g. motor, limit switch, absolute encoder, etc)
      * xxxIO.java E.g. "MotorIO.java" defines the interface/APIs that *all* motors should implement
   * implementations\ - contains hardware specific IO *implementations*. Likely can be re-used, year to year, but might require updates to match vendor library API changes.
      * xxx\ E.g. "Motor" contains all implementations of the "MotorIO" interface
         * xxxIOStub.java E.g. "MotorIOStub.java" - *Stub* implementations contains software-only implementations intended for software testing/simulation.
         * xxxIOAcme.java E.g. "MotorIOAcme.java" - MotorIO interface implementation for the "Acme" branch of motor/controller.
* subsystems\ - subsystem code is contained here.
   * interfaces\ - contains generic subsystem *interface* definitions. Can be re-used, as-is, year to year. "common" commands should be written to use these interfaces
      * xxx.java - E.g. "Intake.java" - defines the generic subsystem interface for an Intake
   * implementations\ - contains implementations subsystem interfaces
      * xxx\ - Contains the one or more *implementations* of a subsystem
         * xxxSubsystem.java E.g. IntakeSubsystemVersionA
* util (?) - TBD
* controls\ (?) - TBD
