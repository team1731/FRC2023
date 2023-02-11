package frc.robot.state.arm;
import frc.data.mp.ArmPath;
import frc.data.mp.ShelfIntake;
import frc.robot.state.StateChangeRequest;

import frc.robot.state.StateWaitCondition;
import frc.robot.state.StateWaitCondition.WaitType;

public class ArmSequenceBuilder {

    private static StateChangeRequest[] getScoreTest() {
        return new StateChangeRequest[]{
            new StateChangeRequest(ArmInput.EXTEND),
            new StateChangeRequest(ArmInput.EXTEND_PING),
            new StateChangeRequest(ArmInput.RELEASE, new StateWaitCondition(ArmWait.UNTIL_LINED_UP_FOR_SCORING, WaitType.PAUSE)),
            new StateChangeRequest(ArmInput.RETRACT),
            new StateChangeRequest(ArmInput.RETRACT_PING)
        };
    }

    private static StateChangeRequest[] getPickupTest() {
        return new StateChangeRequest[]{
            new StateChangeRequest(ArmInput.EXTEND),
            new StateChangeRequest(ArmInput.EXTEND_PING),
            new StateChangeRequest(ArmInput.RETRIEVE),
            new StateChangeRequest(ArmInput.RETRACT),
            new StateChangeRequest(ArmInput.RETRACT_PING)
        };
    }

    private static StateChangeRequest[] getPickup(ArmPath armpath) {
        return new StateChangeRequest[]{
            new StateChangeRequest(ArmInput.EXTEND, armpath),
            new StateChangeRequest(ArmInput.EXTEND_PING),
            new StateChangeRequest(ArmInput.RELEASE),
            new StateChangeRequest(ArmInput.RETRACT),
            new StateChangeRequest(ArmInput.RETRACT_PING)
        };
    }

    private static StateChangeRequest[] getInvalidTest() {
        return new StateChangeRequest[]{
            new StateChangeRequest(ArmInput.EXTEND),
            new StateChangeRequest(ArmInput.EXTEND),
        };
    }

    public static StateChangeRequest[] getSequenceByCode(ArmSequence sequence) {
        switch(sequence) {
            case SCORE_TEST:
                return getScoreTest();

            case PICKUP_TEST:
                return getPickupTest();

            case PICKUP:
                return getPickup(ShelfIntake.getArmPath());

            case INVALID_TEST:
                return getInvalidTest();
                
            default:
            return null;
        
        }
    }
}
