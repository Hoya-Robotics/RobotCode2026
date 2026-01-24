package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * Generic interface for a state based robot subsystem.
 * Relies on a subsystem specific state space enumeration E
 *
 * Implementations should:
 * - call updateState() in their periodic function
 * - implement applyState() to perform hardware/simulation I/O corresponding to the current state
 * - optionally implement handleStateTransitions() if specific actions must be taken between states
 *  ex. moving to certain position to avoid objects
 */
public abstract class StateSubsystem<E extends Enum<E>> extends SubsystemBase {
  private E currentState;
  protected E requestedState;

  public void setState(E state) {
    this.requestedState = state;
  }

  protected final void statePeriodic() {
    this.currentState = handleStateTransitions();
    applyState();
  }

  protected abstract void applyState();

  protected E handleStateTransitions() {
    return requestedState;
  }

  protected final E getRequestedState() {
    return requestedState;
  }

  protected final E getCurrentState() {
    return currentState;
  }
}
