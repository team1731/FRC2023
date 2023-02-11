package frc.robot.state.arm;

import frc.robot.state.*;

/*
 * Defines valid inputs for state transitions
 */
public enum ArmInput implements Input {
    EXTEND, EXTEND_PING, RETRIEVE, RELEASE, RETRACT, 
    RETRACT_PING, RECOVER, INTERRUPT, SUCCESS, FAILED;

    public String getDescription() {
        return "ArmInput: " + this.toString();
    }
}