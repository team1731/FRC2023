package frc.robot.state.arm;

import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;
import frc.robot.state.arm.ArmInput;
import frc.robot.state.arm.ArmState;
import frc.robot.state.arm.IntakeState;
import frc.robot.state.arm.WristState;


public class ArmStateTest {

  @Test 
  void successfulArmTransitions() {
    ArmState state = ArmState.HOME;
    try {
      state = (ArmState)state.next(ArmInput.EXTEND);
      assertEquals(ArmState.EXTENDING, state, "Didn't move to EXTENDING");
      state = (ArmState)state.next(ArmInput.COMPLETED);
      assertEquals(ArmState.EXTENDED, state, "Didn't move to EXTENDED");
      state = (ArmState)state.next(ArmInput.RETRACT);
      assertEquals(ArmState.RETRACTING, state, "Didn't move to RETRACTING");
      state = (ArmState)state.next(ArmInput.COMPLETED);
      assertEquals(ArmState.RETRACTED, state, "Didn't move to RETRACTED");
      state = (ArmState)state.next(ArmInput.RESET);
      assertEquals(ArmState.HOME, state, "Didn't move to HOME");
    } finally {}
  }

  @Test 
  void successfulArmPauseTransitions() {
    ArmState state = ArmState.HOME;
    try {
      state = (ArmState)state.next(ArmInput.EXTEND);
      assertEquals(ArmState.EXTENDING, state, "Didn't move to EXTENDING");
      state = (ArmState)state.next(ArmInput.STOP);
      assertEquals(ArmState.PAUSED, state, "Didn't move to PAUSED");
      state = (ArmState)state.next(ArmInput.EXTEND);
      assertEquals(ArmState.EXTENDING, state, "Didn't move to EXTENDING");
      state = (ArmState)state.next(ArmInput.COMPLETED);
      assertEquals(ArmState.EXTENDED, state, "Didn't move to EXTENDED");
      state = (ArmState)state.next(ArmInput.RETRACT);
      assertEquals(ArmState.RETRACTING, state, "Didn't move to RETRACTING");
      state = (ArmState)state.next(ArmInput.COMPLETED);
      assertEquals(ArmState.RETRACTED, state, "Didn't move to RETRACTED");
      state = (ArmState)state.next(ArmInput.RESET);
      assertEquals(ArmState.HOME, state, "Didn't move to HOME");
    } finally {}
  }

  @Test 
  void successfulIntakeTransitions() {
    IntakeState state = IntakeState.STOPPED;
    try {
      state = (IntakeState)state.next(ArmInput.START);
      assertEquals(IntakeState.RETRIEVING, state, "Didn't move to RETRIEVING");
      state = (IntakeState)state.next(ArmInput.RETRIEVED);
      assertEquals(IntakeState.HOLDING, state, "Didn't move to HOLDING");
      state = (IntakeState)state.next(ArmInput.RELEASE);
      assertEquals(IntakeState.RELEASING, state, "Didn't move to RELEASING");
      state = (IntakeState)state.next(ArmInput.RELEASED);
      assertEquals(IntakeState.STOPPED, state, "Didn't move to RETRACTED");
    } finally {}
  }

  @Test 
  void successfulIntakeStopTransitions() {
    IntakeState state = IntakeState.STOPPED;
    try {
      state = (IntakeState)state.next(ArmInput.START);
      assertEquals(IntakeState.RETRIEVING, state, "Didn't move to RETRIEVING");
      state = (IntakeState)state.next(ArmInput.STOP);
      assertEquals(IntakeState.STOPPED, state, "Didn't move to STOPPED");
      state = (IntakeState)state.next(ArmInput.START);
      assertEquals(IntakeState.RETRIEVING, state, "Didn't move to RETRIEVING");
      state = (IntakeState)state.next(ArmInput.RETRIEVED);
      assertEquals(IntakeState.HOLDING, state, "Didn't move to HOLDING");
      state = (IntakeState)state.next(ArmInput.RELEASE);
      assertEquals(IntakeState.RELEASING, state, "Didn't move to RELEASING");
      state = (IntakeState)state.next(ArmInput.RELEASED);
      assertEquals(IntakeState.STOPPED, state, "Didn't move to RETRACTED");
    } finally {}
  }

  @Test 
  void successfulWristTransitions() {
    WristState state = WristState.HOME;
    try {
      state = (WristState)state.next(ArmInput.FLEX);
      assertEquals(WristState.FLEXED, state, "Didn't move to FLEXED");
      state = (WristState)state.next(ArmInput.RESET);
      assertEquals(WristState.HOME, state, "Didn't move to HOME");
    } finally {}
  }
}