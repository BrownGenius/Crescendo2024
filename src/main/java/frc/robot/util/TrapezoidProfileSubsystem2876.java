// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A subsystem that generates and runs trapezoidal motion profiles automatically. The user specifies
 * how to use the current state of the motion profile by overriding the `useState` method.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public abstract class TrapezoidProfileSubsystem2876 extends SubsystemBase {
  private final double m_period;
  private final TrapezoidProfile m_profile;

  private TrapezoidProfile.State m_state;
  private TrapezoidProfile.State m_goal;

  private boolean m_enabled = true;

  /**
   * Creates a new TrapezoidProfileSubsystem2876.
   *
   * @param constraints The constraints (maximum velocity and acceleration) for the profiles.
   * @param initialPosition The initial position of the controlled mechanism when the subsystem is
   *     constructed.
   * @param period The period of the main robot loop, in seconds.
   */
  public TrapezoidProfileSubsystem2876(
      TrapezoidProfile.Constraints constraints, double initialPosition, double period) {
    requireNonNullParam(constraints, "constraints", "TrapezoidProfileSubsystem2876");
    m_profile = new TrapezoidProfile(constraints);
    m_state = new TrapezoidProfile.State(initialPosition, 0);
    setGoal(initialPosition);
    m_period = period;
  }

  /**
   * Creates a new TrapezoidProfileSubsystem2876.
   *
   * @param constraints The constraints (maximum velocity and acceleration) for the profiles.
   * @param initialPosition The initial position of the controlled mechanism when the subsystem is
   *     constructed.
   */
  public TrapezoidProfileSubsystem2876(
      TrapezoidProfile.Constraints constraints, double initialPosition) {
    this(constraints, initialPosition, 0.02);
  }

  /**
   * Creates a new TrapezoidProfileSubsystem2876.
   *
   * @param constraints The constraints (maximum velocity and acceleration) for the profiles.
   */
  public TrapezoidProfileSubsystem2876(TrapezoidProfile.Constraints constraints) {
    this(constraints, 0, 0.02);
  }

  @Override
  public void periodic() {
    m_state = m_profile.calculate(m_period, m_state, m_goal);
    if (m_enabled) {
      useState(m_state);
    }
  }

  /**
   * Sets the goal state for the subsystem.
   *
   * @param goal The goal state for the subsystem's motion profile.
   */
  public final void setGoal(TrapezoidProfile.State goal) {
    m_goal = goal;
  }

  /**
   * Sets the goal state for the subsystem. Goal velocity assumed to be zero.
   *
   * @param goal The goal position for the subsystem's motion profile.
   */
  public final void setGoal(double goal) {
    setGoal(new TrapezoidProfile.State(goal, 0));
  }

  /** Enable the TrapezoidProfileSubsystem2876's output. */
  public void enable() {
    if (false == m_enabled) {
      // DevilBotz 2876: Update the *current* state before starting the motion
      // profile to ensure smooth motion to the target in case the
      // position/velocity changed *outside* of this subsystem.
      //
      // E.g. robot was disabled and the arm position changed due to gravity
      m_state = getMeasurement();
    }
    m_enabled = true;
  }

  /** Disable the TrapezoidProfileSubsystem2876's output. */
  public void disable() {
    m_enabled = false;
  }

  /**
   * Users should override this to consume the current state of the motion profile.
   *
   * @param state The current state of the motion profile.
   */
  protected abstract void useState(TrapezoidProfile.State state);

  /**
   * Returns the measurement of the process variable used by the TrapezoidProfileSubsystem2876
   *
   * @return the measurement of the process variable
   */
  protected abstract TrapezoidProfile.State getMeasurement();
}
