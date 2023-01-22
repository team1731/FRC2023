package frc.robot.autos;

public class _NotImplementedProperlyException extends Exception {

	private static final long serialVersionUID = 1L;

	public static final String REASON = "You MUST begin your class name with a capital letter followed by a single digit and an underscore!";

	public _NotImplementedProperlyException() {
		super(REASON);
	}

	public _NotImplementedProperlyException(String message) {
		super(message);
	}
}
