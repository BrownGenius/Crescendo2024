# Code Flow
1. [Main.java](Main.java) --> [Robot.java](Robot.java) --> [RobotContainer.java](RobotContainer.java)
   1. Setup Robot Config
      * A persistent preference with the key "Robot Name" is hard-coded onto the RoboRio flash memory
      * The "Robot Name" key is read from the RoboRio at start and determines which robot config to load.
   1. Setup Controls (Bindings)
   1. Setup Autonomous
   