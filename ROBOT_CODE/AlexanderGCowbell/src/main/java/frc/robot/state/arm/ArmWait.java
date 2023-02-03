package frc.robot.state.arm;

import frc.robot.state.Wait;

/*
 * Defines valid wait conditions for the state machine
 */
public enum ArmWait implements Wait {
    UNTIL_LINED_UP_FOR_SCORING, UNTIL_ARM_FINISHED_MOVING;

    public String getDescription() {
        return "ArmWait: " + this.toString();
    }
}
