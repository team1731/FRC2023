package frc.robot.state;

public class StateMachineInitializationException extends Exception {
  private static final String MSG = "State machine failed to initialize";

  public StateMachineInitializationException() {
		super(MSG);
	}

  public StateMachineInitializationException(String identifier) {
		super("[" + identifier + "]: " + MSG);
	}

	public StateMachineInitializationException(String identifier, String message) {
		super("[" + identifier + "] " + MSG + ": " + message);
	}
}
