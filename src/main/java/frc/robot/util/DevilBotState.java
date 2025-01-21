package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

public class DevilBotState {
  public static boolean isRedAlliance() {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isPresent()) {
      return (DriverStation.Alliance.Red == alliance.get());
    }

    return false; // Default to blue alliance if driver station doesn't indicate alliance
  }

  public enum State {
    UNKNOWN,
    DISABLED,
    AUTO,
    TELEOP,
    TEST
  }

  private static State state = State.UNKNOWN;
  private static boolean stateChanged;

  public static void setState(State state) {
    if (state != DevilBotState.state) {
      stateChanged = true;
    }
    DevilBotState.state = state;
  }

  public static State getState() {
    return DevilBotState.state;
  }

  public static boolean stateChanged() {
    if (stateChanged) {
      stateChanged = false;
      return true;
    }
    return false;
  }
}
