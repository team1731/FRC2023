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
      state = (ArmState)state.next(ArmInput.EXTEND_INIT);
      assertEquals(ArmState.EXTEND_REQUESTED, state, "Didn't move to EXTEND_REQUESTED");
      state = (ArmState)state.next(ArmInput.SUCCESS);
      assertEquals(ArmState.EXTEND_INITIALIZED, state, "Didn't move to EXTEND_INITIALIZED");
      state = (ArmState)state.next(ArmInput.EXTEND_MOVE);
      assertEquals(ArmState.EXTEND_MOVE_REQUESTED, state, "Didn't move to EXTEND_MOVE_REQUESTED");
      state = (ArmState)state.next(ArmInput.SUCCESS);
      assertEquals(ArmState.EXTEND_MOVING, state, "Didn't move to EXTEND_MOVING");
      state = (ArmState)state.next(ArmInput.EXTEND_PING);
      assertEquals(ArmState.EXTEND_PINGING, state, "Didn't move to EXTEND_PINGING");
      state = (ArmState)state.next(ArmInput.SUCCESS);
      assertEquals(ArmState.EXTENDED, state, "Didn't move to EXTENDED");
      state = (ArmState)state.next(ArmInput.INTAKE);
      assertEquals(ArmState.RETRIEVING, state, "Didn't move to RETRIEVING");
      state = (ArmState)state.next(ArmInput.SUCCESS);
      assertEquals(ArmState.RETRIEVED, state, "Didn't move to RETRIEVED");
      state = (ArmState)state.next(ArmInput.RETRACT_INIT);
      assertEquals(ArmState.RETRACT_REQUESTED, state, "Didn't move to RETRACT_REQUESTED");
      state = (ArmState)state.next(ArmInput.SUCCESS);
      assertEquals(ArmState.RETRACT_INITIALIZED, state, "Didn't move to RETRACT_INITIALIZED");
      state = (ArmState)state.next(ArmInput.RETRACT_MOVE);
      assertEquals(ArmState.RETRACT_MOVE_REQUESTED, state, "Didn't move to RETRACT_MOVE_REQUESTED");
      state = (ArmState)state.next(ArmInput.SUCCESS);
      assertEquals(ArmState.RETRACT_MOVING, state, "Didn't move to RETRACT_MOVING");
      state = (ArmState)state.next(ArmInput.RETRACT_PING);
      assertEquals(ArmState.RETRACT_PINGING, state, "Didn't move to RETRACT_PINGING");
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
      state = (ArmState)state.next(ArmInput.EXTEND_INIT);
      assertEquals(ArmState.EXTEND_REQUESTED, state, "Didn't move to EXTEND_REQUESTED");
      state = (ArmState)state.next(ArmInput.SUCCESS);
      assertEquals(ArmState.EXTEND_INITIALIZED, state, "Didn't move to EXTEND_INITIALIZED");
      state = (ArmState)state.next(ArmInput.EXTEND_MOVE);
      assertEquals(ArmState.EXTEND_MOVE_REQUESTED, state, "Didn't move to EXTEND_MOVE_REQUESTED");
      state = (ArmState)state.next(ArmInput.SUCCESS);
      assertEquals(ArmState.EXTEND_MOVING, state, "Didn't move to EXTEND_MOVING");
      state = (ArmState)state.next(ArmInput.EXTEND_PING);
      assertEquals(ArmState.EXTEND_PINGING, state, "Didn't move to EXTEND_PINGING");
      state = (ArmState)state.next(ArmInput.SUCCESS);
      assertEquals(ArmState.EXTENDED, state, "Didn't move to EXTENDED");
      state = (ArmState)state.next(ArmInput.RELEASE);
      assertEquals(ArmState.RELEASING, state, "Didn't move to RELEASING");
      state = (ArmState)state.next(ArmInput.SUCCESS);
      assertEquals(ArmState.RELEASED, state, "Didn't move to RELEASED");
      state = (ArmState)state.next(ArmInput.RETRACT_INIT);
      assertEquals(ArmState.RETRACT_REQUESTED, state, "Didn't move to RETRACT_REQUESTED");
      state = (ArmState)state.next(ArmInput.SUCCESS);
      assertEquals(ArmState.RETRACT_INITIALIZED, state, "Didn't move to RETRACT_INITIALIZED");
      state = (ArmState)state.next(ArmInput.RETRACT_MOVE);
      assertEquals(ArmState.RETRACT_MOVE_REQUESTED, state, "Didn't move to RETRACT_MOVE_REQUESTED");
      state = (ArmState)state.next(ArmInput.SUCCESS);
      assertEquals(ArmState.RETRACT_MOVING, state, "Didn't move to RETRACT_MOVING");
      state = (ArmState)state.next(ArmInput.RETRACT_PING);
      assertEquals(ArmState.RETRACT_PINGING, state, "Didn't move to RETRACT_PINGING");
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
      state = (ArmState)state.next(ArmInput.EXTEND_INIT);
      assertEquals(ArmState.EXTEND_REQUESTED, state, "Didn't move to EXTENDING");
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
      state = (ArmState)state.next(ArmInput.EXTEND_INIT);
      assertEquals(ArmState.EXTEND_REQUESTED, state, "Didn't move to EXTENDING");
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
      state = (ArmState)state.next(ArmInput.EXTEND_INIT);
      assertEquals(ArmState.EXTEND_REQUESTED, state, "Didn't move to EXTENDING");
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
