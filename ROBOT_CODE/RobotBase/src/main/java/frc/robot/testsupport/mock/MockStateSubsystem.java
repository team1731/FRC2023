package frc.robot.testsupport.mock;

import frc.robot.state.*;
import frc.robot.state.arm.ArmStateMachine.ArmInput;
import frc.robot.Constants.StateConstants;
import edu.wpi.first.wpilibj.Timer;

public class MockStateSubsystem implements StateHandler {
    private StateMachine stateMachine;
    public boolean wasInterrupted = false;
    public boolean shouldFail = false;

    public void registerStateMachine(StateMachine stateMachine) {
      this.stateMachine = stateMachine;
    }

    public void changeState(Input input, Object data) {
      StateChangeResult result;
      if(shouldFail) {
        result = new StateChangeResult(StateConstants.kGenericFailedCode, "Failed", Timer.getFPGATimestamp());
      } else {
        result = new StateChangeResult(StateConstants.kSuccessCode, "Success", Timer.getFPGATimestamp());
      }
      stateMachine.transition(ArmInput.SUCCESS, result);
    }

    public void interruptStateChange() {
      wasInterrupted = true;
    }
}
