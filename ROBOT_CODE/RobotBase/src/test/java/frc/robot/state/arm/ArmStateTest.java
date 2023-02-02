package frc.robot.state.arm;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.fail;
import org.junit.jupiter.api.Test;
import frc.robot.state.arm.ArmInput;
import frc.robot.state.arm.ArmState;
import frc.robot.state.*;


public class ArmStateTest {
  @Test 
  void successfulScoringTransitions() {
    ArmState state = ArmState.RETRACTED;
    try {
      state = (ArmState)state.next(ArmInput.EXTEND);
      assertEquals(ArmState.EXTENDING, state, "Didn't move to EXTENDING");
      state = (ArmState)state.next(ArmInput.SUCCESS);
      assertEquals(ArmState.EXTENDED, state, "Didn't move to EXTENDED");
      state = (ArmState)state.next(ArmInput.RELEASE);
      assertEquals(ArmState.RELEASING, state, "Didn't move to RELEASING");
      state = (ArmState)state.next(ArmInput.SUCCESS);
      assertEquals(ArmState.RELEASED, state, "Didn't move to RELEASED");
      state = (ArmState)state.next(ArmInput.RETRACT);
      assertEquals(ArmState.RETRACTING, state, "Didn't move to RETRACTING");
      state = (ArmState)state.next(ArmInput.SUCCESS);
      assertEquals(ArmState.RETRACTED, state, "Didn't move to RETRACTED");
    } catch(StateMachineInvalidTransitionException ite) {
      fail(ite);
    }
  }

  @Test 
  void successfulPickupTransitions() {
    ArmState state = ArmState.RETRACTED;
    try {
      state = (ArmState)state.next(ArmInput.EXTEND);
      assertEquals(ArmState.EXTENDING, state, "Didn't move to EXTENDING");
      state = (ArmState)state.next(ArmInput.SUCCESS);
      assertEquals(ArmState.EXTENDED, state, "Didn't move to EXTENDED");
      state = (ArmState)state.next(ArmInput.INTAKE);
      assertEquals(ArmState.RETRIEVING, state, "Didn't move to RETRIEVING");
      state = (ArmState)state.next(ArmInput.SUCCESS);
      assertEquals(ArmState.RETRIEVED, state, "Didn't move to RETRIEVED");
      state = (ArmState)state.next(ArmInput.RETRACT);
      assertEquals(ArmState.RETRACTING, state, "Didn't move to RETRACTING");
      state = (ArmState)state.next(ArmInput.SUCCESS);
      assertEquals(ArmState.RETRACTED, state, "Didn't move to RETRACTED");
    } catch(StateMachineInvalidTransitionException ite) {
      fail(ite);
    }
  }

  @Test 
  void interruptedTransitionHandling() {
    ArmState state = ArmState.RETRACTED;
    try {
      state = (ArmState)state.next(ArmInput.EXTEND);
      assertEquals(ArmState.EXTENDING, state, "Didn't move to EXTENDING");
      state = (ArmState)state.next(ArmInput.INTERRUPT);
      assertEquals(ArmState.INTERRUPTED, state, "Didn't move to INTERRUPTED");
      state = (ArmState)state.next(ArmInput.RECOVER);
      assertEquals(ArmState.RECOVERING, state, "Didn't move to RECOVERING");
      state = (ArmState)state.next(ArmInput.SUCCESS);
      assertEquals(ArmState.RETRACTED, state, "Didn't move to RETRACTED");
    } catch(StateMachineInvalidTransitionException ite) {
      fail(ite);
    }
  }

  @Test 
  void failedTransitionHandling() {
    ArmState state = ArmState.RETRACTED;
    try {
      state = (ArmState)state.next(ArmInput.EXTEND);
      assertEquals(ArmState.EXTENDING, state, "Didn't move to EXTENDING");
      state = (ArmState)state.next(ArmInput.FAILED);
      assertEquals(ArmState.INTERRUPTED, state, "Didn't move to INTERRUPTED");
      state = (ArmState)state.next(ArmInput.RECOVER);
      assertEquals(ArmState.RECOVERING, state, "Didn't move to RECOVERING");
      state = (ArmState)state.next(ArmInput.SUCCESS);
      assertEquals(ArmState.RETRACTED, state, "Didn't move to RETRACTED");
    } catch(StateMachineInvalidTransitionException ite) {
      fail(ite);
    }
  }

  @Test 
  void unsafeTransitionHandling() {
    ArmState state = ArmState.RETRACTED;
    try {
      state = (ArmState)state.next(ArmInput.EXTEND);
      assertEquals(ArmState.EXTENDING, state, "Didn't move to EXTENDING");
      state = (ArmState)state.next(ArmInput.FAILED);
      assertEquals(ArmState.INTERRUPTED, state, "Didn't move to INTERRUPTED");
      state = (ArmState)state.next(ArmInput.RECOVER);
      assertEquals(ArmState.RECOVERING, state, "Didn't move to RECOVERING");
      state = (ArmState)state.next(ArmInput.FAILED);
      assertEquals(ArmState.UNSAFE, state, "Didn't move to UNSAFE");
    } catch(StateMachineInvalidTransitionException ite) {
      fail(ite);
    }
  }
}
