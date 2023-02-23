package frc.robot.state.arm;

import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;
import frc.robot.state.arm.ArmStateMachine.Input;

public class IntakeStateTest {

  @Test 
  void successfulIntakeTransitions() {
    IntakeState state = IntakeState.STOPPED;
    try {
      state = (IntakeState)state.next(Input.START);
      assertEquals(IntakeState.STARTING, state, "Didn't move to STARTING");
      state = (IntakeState)state.next(Input.STARTED);
      assertEquals(IntakeState.RETRIEVING, state, "Didn't move to RETRIEVING");
      state = (IntakeState)state.next(Input.RETRIEVED);
      assertEquals(IntakeState.HOLDING, state, "Didn't move to HOLDING");
      state = (IntakeState)state.next(Input.RELEASE);
      assertEquals(IntakeState.RELEASING, state, "Didn't move to RELEASING");
      state = (IntakeState)state.next(Input.RELEASED);
      assertEquals(IntakeState.STOPPED, state, "Didn't move to RETRACTED");
    } finally {}
  }

  @Test 
  void successfulIntakeStopTransitions() {
    IntakeState state = IntakeState.STOPPED;
    try {
      state = (IntakeState)state.next(Input.START);
      assertEquals(IntakeState.STARTING, state, "Didn't move to STARTING");
      state = (IntakeState)state.next(Input.STARTED);
      assertEquals(IntakeState.RETRIEVING, state, "Didn't move to RETRIEVING");
      state = (IntakeState)state.next(Input.STOP);
      assertEquals(IntakeState.STOPPED, state, "Didn't move to STOPPED");
    } finally {}
  }
  
}