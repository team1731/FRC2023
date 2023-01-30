package frc.robot.state;

public class StateMachineInvalidTransitionException extends Exception {
  private static final String MSG = "State machine encountered an invalid transition";

  public StateMachineInvalidTransitionException() {
		super(MSG);
	}

  public StateMachineInvalidTransitionException(String identifier) {
		super("[" + identifier + "]: " + MSG);
	}

	public StateMachineInvalidTransitionException(String identifier, String message) {
		super("[" + identifier + "] " + MSG + ": " + message);
	}
}
