package frc.robot.testsupport.mock;

import frc.robot.state.*;
import frc.robot.state.arm.ArmInput;
import frc.robot.Constants.StateConstants.ResultCode;
import edu.wpi.first.wpilibj.Timer;

public class MockStateSubsystem implements StateHandler {
    private StateMachine stateMachine;
    public boolean wasInterrupted = false;
    public boolean shouldFail = false;

    public void registerStateMachine(StateMachine stateMachine) {
      this.stateMachine = stateMachine;
    }

    public void changeState(Input input, Object data) {
      if(shouldFail && input != ArmInput.RECOVER) {
        notifyStateMachine(ArmInput.FAILED, ResultCode.FAILED, "Failed");
      } else {
        notifyStateMachine(ArmInput.SUCCESS, ResultCode.SUCCESS, "Success");
      }
    }

    private void notifyStateMachine(Input input, ResultCode resultCode, String resultMessage) {
      StateChangeResult result = new StateChangeResult(resultCode, resultMessage, Timer.getFPGATimestamp());
      stateMachine.transition(input, result);
    }

    public void interruptStateChange() {
      wasInterrupted = true;
    }

    public void periodic() {
      notifyStateMachine(ArmInput.SUCCESS, ResultCode.SUCCESS, "Success");
    }
}
