package frc.robot.state.arm;

import frc.robot.state.*;

/*
 * Defines valid inputs for state transitions
 */
public enum ArmInput implements Input {
    EXTEND, RETRACT, INTAKE, RELEASE, RECOVER, INTERRUPT, SUCCESS, FAILED;

    public String getDescription() {
        return "ArmInput: " + this.toString();
    }
}
