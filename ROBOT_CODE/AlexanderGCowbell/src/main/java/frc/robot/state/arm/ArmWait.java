package frc.robot.state.arm;

import frc.robot.state.Wait;

/*
 * Defines valid wait conditions for the state machine
 */
public enum ArmWait implements Wait {
    UNTIL_LINED_UP_FOR_SCORING,     // waiting until driver gives OK to release
    UNTIL_LINED_UP_FOR_PICKUP,      // waiting until driver gives OK to pickup
    UNTIL_ARM_FINISHED_MOVING;      // waiting until the motors finish processing motion profile

    public String getDescription() {
        return "ArmWait: " + this.toString();
    }
}
