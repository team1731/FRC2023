package frc.robot.state;

public interface StateHandler {
  public void registerStateMachine(StateMachine stateMachine);
  public void changeState(Input input, Object data);
  public void interruptStateChange();
  public void periodic();
}
