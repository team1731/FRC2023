package frc.robot.state.arm;

import frc.robot.state.*;

/*
 * Defines valid sequences that can be supplied to the state machine
 * Note: leave test sequences in place, they are used by the ArmStateMachineTest (JUnit)
 */
public enum ArmSequence implements Sequence {
    SCORE_TEST("ABC"), 
    PICKUP_TEST("XYZ"), 
    PICKUP("123"),
    UNDEFINED_TEST(null), 
    INVALID_TEST(null);

    public final String code;

    private ArmSequence(String code) {
        this.code = code;
    }

    public static ArmSequence valueForCode(String code) {
        for (ArmSequence as : values()) {
            if (as.code.equals(code)) {
                return as;
            }
        }
        return null;
    }

    public String getDescription() {
        return "ArmSequence: " + this.toString();
    }
};
