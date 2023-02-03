package frc.robot.state.arm;

import frc.robot.state.*;

/*
 * Defines valid sequences that can be supplied to the state machine
 * Note: leave test sequences in place, they are used by the ArmStateMachineTest (JUnit)
 */
public enum ArmSequence implements StateSequence {
    SCORE_TEST, PICKUP_TEST, UNDEFINED_TEST, INVALID_TEST;

    public String getDescription() {
        return "ArmSequence: " + this.toString();
    }
};
