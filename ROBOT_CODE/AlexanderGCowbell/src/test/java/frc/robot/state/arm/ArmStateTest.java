package frc.robot.state.arm;

import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;
import frc.robot.state.arm.ArmStateMachine.Input;


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
      assertEquals(ArmState.RESETTING, state, "Didn't move to RESETTING");
      state = (ArmState)state.next(Input.COMPLETED);
      /* 
      assertEquals(ArmState.RESETTING_WRIST, state, "Didn't move to RESETTING_WRIST");
      state = (ArmState)state.next(Input.COMPLETED);
      assertEquals(ArmState.RESETTING_PROXIMAL, state, "Didn't move to RESETTING_PROXIMAL");
      state = (ArmState)state.next(Input.COMPLETED);
      assertEquals(ArmState.RESETTING_DISTAL, state, "Didn't move to RESETTING_DISTAL");
      state = (ArmState)state.next(Input.COMPLETED);
      */
      assertEquals(ArmState.HOME, state, "Didn't move to HOME");
    } finally {}
  }
  
  @Test 
  void successfulArmInitializeTransitions() {
    ArmState state = ArmState.UNKNOWN;
    try {
      state = (ArmState)state.next(Input.INITIALIZE);
      assertEquals(ArmState.RESETTING, state, "Didn't move to RESETTING");
      state = (ArmState)state.next(Input.COMPLETED);
      /*
      assertEquals(ArmState.RESETTING_WRIST, state, "Didn't move to RESETTING_WRIST");
      state = (ArmState)state.next(Input.COMPLETED);
      assertEquals(ArmState.RESETTING_PROXIMAL, state, "Didn't move to RESETTING_PROXIMAL");
      state = (ArmState)state.next(Input.COMPLETED);
      assertEquals(ArmState.RESETTING_DISTAL, state, "Didn't move to RESETTING_DISTAL");
      state = (ArmState)state.next(Input.COMPLETED);
      */
      assertEquals(ArmState.HOME, state, "Didn't move to HOME");
      state = (ArmState)state.next(Input.EXTEND);
      assertEquals(ArmState.EXTENDING, state, "Didn't move to EXTENDING");
      state = (ArmState)state.next(Input.COMPLETED);
      assertEquals(ArmState.EXTENDED, state, "Didn't move to EXTENDED");
      state = (ArmState)state.next(Input.RETRACT);
      assertEquals(ArmState.RETRACTING, state, "Didn't move to RETRACTING");
      state = (ArmState)state.next(Input.COMPLETED);
      assertEquals(ArmState.RESETTING, state, "Didn't move to RESETTING");
      state = (ArmState)state.next(Input.COMPLETED);
      /* 
      assertEquals(ArmState.RESETTING_WRIST, state, "Didn't move to RESETTING_WRIST");
      state = (ArmState)state.next(Input.COMPLETED);
      assertEquals(ArmState.RESETTING_PROXIMAL, state, "Didn't move to RESETTING_PROXIMAL");
      state = (ArmState)state.next(Input.COMPLETED);
      assertEquals(ArmState.RESETTING_DISTAL, state, "Didn't move to RESETTING_DISTAL");
      state = (ArmState)state.next(Input.COMPLETED);
      */
      assertEquals(ArmState.HOME, state, "Didn't move to HOME");
    } finally {}
  }
  
}