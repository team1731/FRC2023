package frc.robot.state.arm;

import frc.robot.state.*;

/*
 * Defines valid inputs for state transitions
 */
public enum ArmInput implements Input {
    EXTEND_INIT, EXTEND_MOVE, EXTEND_PING, INTAKE, RELEASE, RETRACT_INIT, 
    RETRACT_MOVE, RETRACT_PING, RECOVER, INTERRUPT, SUCCESS, FAILED;

    public String getDescription() {
        return "ArmStateInput: " + this.toString();
    }
}
