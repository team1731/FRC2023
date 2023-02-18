package frc.robot.state.arm;

/*
 * Defines valid inputs for state transitions
 */
public enum ArmInput {
    EXTEND, COMPLETED, RETRACT, RESET, START, STOP, INTERRUPT,
    RETRIEVED, RELEASE, RELEASED, FLEX;

    public String getDescription() {
        return "ArmInput: " + this.toString();
    }
}