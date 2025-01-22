package frc.robot.subsystems.controls.vision;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.interfaces.Vision;
import frc.robot.subsystems.interfaces.Vision.Camera;
import java.util.Map;

public class VisionControls {
  public static void addGUI(Vision vision, ShuffleboardTab tab) {
    for (Camera camera : vision.getCameras()) {
      try {
        tab.addCamera(
                camera.getName() + " cam",
                camera.getName(),
                "mjpg:http://10.28.76.11:" + camera.getPort() + "/?action=stream")
            .withProperties(Map.of("showControls", false))
            //              .withPosition(colIndex, rowIndex)
            .withSize(3, 3);
      } catch (Exception e) {
        // Do Nothing
      }
      //        rowIndex += 3;
    }
  }
}
