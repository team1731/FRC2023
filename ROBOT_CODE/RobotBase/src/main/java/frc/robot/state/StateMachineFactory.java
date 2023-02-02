package frc.robot.state;

import frc.robot.Constants.StateConstants;
import frc.robot.state.arm.ArmStateMachine;

public final class StateMachineFactory {
  private static StateMachineFactory instance;
  private ArmStateMachine armStateMachine;

  private StateMachineFactory() {
  }

  public static StateMachineFactory getInstance() {
    if(instance == null) {
      instance = new StateMachineFactory();
    }
    return instance;
  }

  public ArmStateMachine getArmStateMachine(StateHandler stateHandler) {
    if(armStateMachine == null) {
      armStateMachine = new ArmStateMachine(StateConstants.kArmStateMachineId, stateHandler);
    }
    return armStateMachine;
  }
}
