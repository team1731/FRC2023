package frc.robot.tests;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.fail;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.state.*;
import frc.robot.state.StateMachine.Status;
import frc.robot.state.arm.ArmStateMachine;
import frc.robot.state.arm.ArmStateMachine.ArmInput;
import frc.robot.state.arm.ArmStateMachine.ArmState;
import frc.robot.state.arm.ArmStateMachine.ArmSequence;
import frc.robot.Constants.StateConstants;
import frc.robot.Constants.StateConstants.StateMachineWaitCondition;
import frc.robot.testsupport.mock.MockStateSubsystem;

class ArmStateMachineTest {
  ArmStateMachine stateMachine;
  MockStateSubsystem mockSubsystem;
  
  @BeforeEach // this method will run before each test
  void setup() {
    mockSubsystem = new MockStateSubsystem();
    stateMachine = StateMachineFactory.getInstance().getArmStateMachine(mockSubsystem);
  }

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

  @Test 
  void successfulScoreSequence() {
    try {
      stateMachine.initialize(ArmSequence.SCORE_TEST);
    } catch(StateMachineInitializationException ste) {
      ste.printStackTrace();
    }

    assertEquals(StateConstants.kArmStateMachineId, stateMachine.getId(), "Arm machine ID doesn't match");
    assertEquals(Status.READY, stateMachine.getStatus(), "Status should be READY");

    // start sequence and let it run until wait condition
    stateMachine.startSequence();

    assertEquals(Status.WAITING, stateMachine.getStatus(), "Status should be WAITING");

    // release the wait condition
    stateMachine.resolveWaitCondition(StateMachineWaitCondition.UNTIL_LINED_UP_FOR_SCORING);
    
    // continue the process to the end
    stateMachine.periodic();

    assertEquals(Status.FINISHED, stateMachine.getStatus(), "Status should be FINISHED");

    // grab all processed steps
    var processedSteps = stateMachine.getProcessedSteps().toArray();
    assertEquals(6, processedSteps.length, "Wrong number of processed steps");
    // Step 1
    StateChange step = (StateChange)processedSteps[0];
    assertEquals(step.previousState, ArmState.RETRACTED, "[0] Previous state should be RETRACTED");
    assertEquals(step.newState, ArmState.EXTENDING, "[0] New state should be EXTENDING");
    assertEquals(step.request.input, ArmInput.EXTEND, "[0] Request input should be EXTEND");
    // Step 2
    step = (StateChange)processedSteps[1];
    assertEquals(step.previousState, ArmState.EXTENDING, "[1] Previous state should be EXTENDING");
    assertEquals(step.newState, ArmState.EXTENDED, "[1] New state should be EXTENDED");
    assertEquals(step.request.input, ArmInput.SUCCESS, "[1] Request input should be SUCCESS");
    // Step 3
    step = (StateChange)processedSteps[2];
    assertEquals(step.previousState, ArmState.EXTENDED, "[2] Previous state should be EXTENDED");
    assertEquals(step.newState, ArmState.RELEASING, "[2] New state should be RELEASING");
    assertEquals(step.request.input, ArmInput.RELEASE, "[2] Request input should be RELEASE");
    // Step 4
    step = (StateChange)processedSteps[3];
    assertEquals(step.previousState, ArmState.RELEASING, "[3] Previous state should be RELEASING");
    assertEquals(step.newState, ArmState.RELEASED, "[3] New state should be RELEASED");
    assertEquals(step.request.input, ArmInput.SUCCESS, "[3] Request input should be SUCCESS");
    // Step 5
    step = (StateChange)processedSteps[4];
    assertEquals(step.previousState, ArmState.RELEASED, "[4] Previous state should be RELEASED");
    assertEquals(step.newState, ArmState.RETRACTING, "[4] New state should be RETRACTING");
    assertEquals(step.request.input, ArmInput.RETRACT, "[4] Request input should be RETRACT");
    // Step 6
    step = (StateChange)processedSteps[5];
    assertEquals(step.previousState, ArmState.RETRACTING, "[5] Previous state should be RETRACTING");
    assertEquals(step.newState, ArmState.RETRACTED, "[5] New state should be RETRACTED");
    assertEquals(step.request.input, ArmInput.SUCCESS, "[5] Request input should be SUCCESS");
  }

  @Test 
  void successfulPickupSequence() {
    fail("NOT IMPLEMENTED");
    //Status Ready, Running, Waiting, Finished, Invalid
    //ID
    try {
      stateMachine.initialize(ArmSequence.SCORE_TEST);
    } catch(StateMachineInitializationException ste) {
      ste.printStackTrace();
    }

    // start sequence and let it run until wait condition
    stateMachine.startSequence();

    // release the wait condition
    stateMachine.resolveWaitCondition(StateMachineWaitCondition.UNTIL_LINED_UP_FOR_SCORING);
    
    // continue the process to the end
    stateMachine.periodic();

    
  }

  /*
   * Failed initialization
    Interruption and recovery
    Failed score path and recovery
    Failed pickup path and recovery
    //Status Running, Invalid
   */
}
