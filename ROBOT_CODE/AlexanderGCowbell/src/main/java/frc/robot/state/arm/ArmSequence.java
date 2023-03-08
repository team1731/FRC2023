package frc.robot.state.arm;

import frc.robot.Constants.OperatorConsoleConstants;

/*
 * Note: if the sequence has a keypad mapping the appropriate keypad input should be 
 * set for the 'code' and otherwise the code should be left null
 */

public enum ArmSequence {
    READ_OPERATOR_ENTRY,
    SCORE_HIGH("DEPLOY_HIGH", OperatorConsoleConstants.kScoreHighSwitchId),
    SCORE_MEDIUM("DEPLOY_MID", OperatorConsoleConstants.kScoreMediumSwitchId),
    SCORE_LOW("DEPLOY_LOW", OperatorConsoleConstants.kScoreLowSwitchId),
    PICKUP_HIGH,
    PICKUP_LOW,
    PICKUP_DOWNED_CONE,
    FLIP_CONE;

    public String code = null;  // used if operator inputting w/keypad
    public int switchId = -1;   // used if operator inputting w/switch on flight controller

    private ArmSequence() {
    }

    private ArmSequence(String code, int switchId) {
        this.code = code;
        this.switchId = switchId;
    }

    public static ArmSequence valueForCode(String code) {
        for(ArmSequence as : values()) {
            if(as.code != null && as.code.equals(code)) {
                return as;
            }
        }
        return null;
    }

    public static ArmSequence valueForSwitch(int switchId) {
        for(ArmSequence as : values()) {
            if(as.switchId != -1 && as.switchId == switchId) {
                return as;
            }
        }
        return null;
    }
};
