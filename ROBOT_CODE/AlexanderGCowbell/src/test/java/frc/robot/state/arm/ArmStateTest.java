package frc.robot.state.arm;

import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;
import frc.robot.state.arm.ArmStateMachine.Input;
import frc.robot.state.arm.ArmState;
import frc.robot.state.arm.IntakeState;


public class ArmStateTest {

  
  @Test 
  void successfulArmTransitions() {
    ArmState state = ArmState.HOME;
    try {
      state = (ArmState)state.next(Input.EXTEND);
      assertEquals(ArmState.EXTENDING, state, "Didn't move to EXTENDING");
      state = (ArmState)state.next(Input.COMPLETED);
      assertEquals(ArmState.EXTENDED, state, "Didn't move to EXTENDED");
      state = (ArmState)state.next(Input.RETRACT);
      assertEquals(ArmState.RETRACTING, state, "Didn't move to RETRACTING");
      state = (ArmState)state.next(Input.COMPLETED);
      assertEquals(ArmState.RESETTING_WRIST, state, "Didn't move to RESETTING_WRIST");
      state = (ArmState)state.next(Input.COMPLETED);
      assertEquals(ArmState.RESETTING_PROXIMAL, state, "Didn't move to RESETTING_PROXIMAL");
      state = (ArmState)state.next(Input.COMPLETED);
      assertEquals(ArmState.RESETTING_DISTAL, state, "Didn't move to RESETTING_DISTAL");
      state = (ArmState)state.next(Input.COMPLETED);
      assertEquals(ArmState.HOME, state, "Didn't move to HOME");
    } finally {}
  }

  @Test 
  void successfulIntakeTransitions() {
    IntakeState state = IntakeState.STOPPED;
    try {
      state = (IntakeState)state.next(Input.START);
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
      assertEquals(IntakeState.RETRIEVING, state, "Didn't move to RETRIEVING");
      state = (IntakeState)state.next(Input.STOP);
      assertEquals(IntakeState.STOPPED, state, "Didn't move to STOPPED");
      state = (IntakeState)state.next(Input.START);
      assertEquals(IntakeState.RETRIEVING, state, "Didn't move to RETRIEVING");
      state = (IntakeState)state.next(Input.RETRIEVED);
      assertEquals(IntakeState.HOLDING, state, "Didn't move to HOLDING");
      state = (IntakeState)state.next(Input.RELEASE);
      assertEquals(IntakeState.RELEASING, state, "Didn't move to RELEASING");
      state = (IntakeState)state.next(Input.RELEASED);
      assertEquals(IntakeState.STOPPED, state, "Didn't move to RETRACTED");
    } finally {}
  }
  
}