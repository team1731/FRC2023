package frc.robot.state;

public interface State {
  public State next(Input input) throws StateMachineInvalidTransitionException;
  public String getDescription();
}
