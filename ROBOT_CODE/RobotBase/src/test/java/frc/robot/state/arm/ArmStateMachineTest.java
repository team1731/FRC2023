package frc.robot.state.arm;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import frc.robot.state.*;
import frc.robot.state.StateMachine.Status;
import frc.robot.state.arm.ArmStateMachine;
import frc.robot.state.arm.ArmInput;
import frc.robot.state.arm.ArmState;
import frc.robot.state.arm.ArmSequence;
import frc.robot.Constants.StateConstants;
import frc.robot.testsupport.mock.MockStateSubsystem;

class ArmStateMachineTest {
  ArmStateMachine stateMachine;
  MockStateSubsystem mockSubsystem;
  
  @BeforeEach // this method will run before each test
  void setup() {
    mockSubsystem = new MockStateSubsystem();
    stateMachine = StateMachineFactory.getInstance().getArmStateMachine(mockSubsystem);
  }

  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
    stateMachine.reset();
  }

  /* 
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

    assertEquals(Status.PAUSED, stateMachine.getStatus(), "Status should be PAUSED");

    // release the wait condition
    stateMachine.resolveWaitCondition(ArmWait.UNTIL_LINED_UP_FOR_SCORING);
    
    // continue the process to the end
    stateMachine.periodic();

    assertEquals(Status.FINISHED, stateMachine.getStatus(), "Status should be FINISHED");

    // grab all processed steps
    var processedSteps = stateMachine.getProcessedSteps().toArray();
    assertEquals(6, processedSteps.length, "Wrong number of processed steps");
    // Step 1
    StateChange step = (StateChange)processedSteps[0];
    assertEquals(step.previousState, ArmState.RETRACTED, "[0] Previous state should be RETRACTED");
    assertEquals(step.newState, ArmState.EXTEND_REQUESTED, "[0] New state should be EXTEND_REQUESTED");
    assertEquals(step.request.input, ArmInput.EXTEND_INIT, "[0] Request input should be EXTEND_INIT");
    // Step 2
    step = (StateChange)processedSteps[1];
    assertEquals(step.previousState, ArmState.EXTEND_REQUESTED, "[1] Previous state should be EXTEND_REQUESTED");
    assertEquals(step.newState, ArmState.EXTEND_INITIALIZED, "[1] New state should be EXTEND_INITIALIZED");
    assertEquals(step.result.code, StateConstants.kSuccessCode, "[1] Result should indicate success");
    // Step 3
    step = (StateChange)processedSteps[2];
    assertEquals(step.previousState, ArmState.EXTEND_INITIALIZED, "[2] Previous state should be EXTEND_INITIALIZED");
    assertEquals(step.newState, ArmState.EXTEND_MOVE_REQUESTED, "[2] New state should be EXTEND_MOVE_REQUESTED");
    assertEquals(step.request.input, ArmInput.EXTEND_MOVE, "[2] Request input should be EXTEND_MOVE");
    // Step 4
    step = (StateChange)processedSteps[3];
    assertEquals(step.previousState, ArmState.EXTEND_MOVE_REQUESTED, "[3] Previous state should be EXTEND_MOVE_REQUESTED");
    assertEquals(step.newState, ArmState.EXTEND_MOVING, "[3] New state should be EXTEND_MOVE");
    assertEquals(step.result.code, StateConstants.kSuccessCode, "[3] Result should indicate success");
    // Step 5
    step = (StateChange)processedSteps[4];
    assertEquals(step.previousState, ArmState.EXTEND_MOVING, "[4] Previous state should be EXTEND_MOVING");
    assertEquals(step.newState, ArmState.EXTEND_PINGING, "[4] New state should be EXTEND_PINGING");
    assertEquals(step.request.input, ArmInput.EXTEND_PING, "[4] Request input should be EXTEND_PING");
    // Step 6
    step = (StateChange)processedSteps[5];
    assertEquals(step.previousState, ArmState.EXTEND_PINGING, "[5] Previous state should be EXTEND_PINGING");
    assertEquals(step.newState, ArmState.EXTENDED, "[5] New state should be EXTENDED");
    assertEquals(step.result.code, StateConstants.kSuccessCode, "[5] Result should indicate success");
  }

  @Test 
  void successfulPickupSequence() {
    try {
      stateMachine.initialize(ArmSequence.PICKUP_TEST);
    } catch(StateMachineInitializationException ste) {
      ste.printStackTrace();
    }

    assertEquals(StateConstants.kArmStateMachineId, stateMachine.getId(), "Arm machine ID doesn't match");
    assertEquals(Status.READY, stateMachine.getStatus(), "Status should be READY");

    // start sequence and let it run until finished
    stateMachine.startSequence();
    assertEquals(Status.FINISHED, stateMachine.getStatus(), "Status should be FINISHED");

    // grab all processed steps
    var processedSteps = stateMachine.getProcessedSteps().toArray();
    assertEquals(6, processedSteps.length, "Wrong number of processed steps");
    // Step 1
    StateChange step = (StateChange)processedSteps[0];
    assertEquals(step.previousState, ArmState.EXTEND_MOVING, "[0] Previous state should be EXTEND_MOVING");
    assertEquals(step.newState, ArmState.EXTEND_PINGING, "[0] New state should be EXTEND_PINGING");
    assertEquals(step.request.input, ArmInput.EXTEND_PING, "[0] Request input should be EXTEND_PING");
    // Step 2
    step = (StateChange)processedSteps[1];
    assertEquals(step.previousState, ArmState.EXTEND_PINGING, "[1] Previous state should be EXTEND_PINGING");
    assertEquals(step.newState, ArmState.EXTENDED, "[1] New state should be EXTENDED");
    assertEquals(step.result.code, StateConstants.kSuccessCode, "[1] Result should indicate success");
    // Step 3
    step = (StateChange)processedSteps[2];
    assertEquals(step.previousState, ArmState.EXTENDED, "[2] Previous state should be EXTENDED");
    assertEquals(step.newState, ArmState.RETRIEVING, "[2] New state should be RETRIEVING");
    assertEquals(step.request.input, ArmInput.INTAKE, "[2] Request input should be INTAKE");
    // Step 4
    step = (StateChange)processedSteps[3];
    assertEquals(step.previousState, ArmState.RETRIEVING, "[3] Previous state should be RETRIEVING");
    assertEquals(step.newState, ArmState.RETRIEVED, "[3] New state should be RETRIEVED");
    assertEquals(step.result.code, StateConstants.kSuccessCode, "[3] Result should indicate success");
    // Step 5
    step = (StateChange)processedSteps[4];
    assertEquals(step.previousState, ArmState.RETRIEVED, "[4] Previous state should be RETRIEVED");
    assertEquals(step.newState, ArmState.RETRACT_REQUESTED, "[4] New state should be RETRACT_REQUESTED");
    assertEquals(step.request.input, ArmInput.RETRACT_INIT, "[4] Request input should be RETRACT_INIT");
    // Step 6
    step = (StateChange)processedSteps[5];
    assertEquals(step.previousState, ArmState.RETRACT_REQUESTED, "[5] Previous state should be RETRACT_REQUESTED");
    assertEquals(step.newState, ArmState.RETRACT_INITIALIZED, "[5] New state should be RETRACT_INITIALIZED");
    assertEquals(step.result.code, StateConstants.kSuccessCode, "[5] Result should indicate success");
  }

  @Test 
  void failedInitialization() {
    try {
      stateMachine.initialize(ArmSequence.UNDEFINED_TEST);
      fail("Initiazation exception was not thrown");
    } catch(StateMachineInitializationException ste) {
      assertEquals(stateMachine.getStatus(), Status.FAILED_INIT, "Status should be FAILED_INIT");
    }
  }

  @Test 
  void invalidSequence() {
    // intentionally initiating an exception, suppress reporting
    stateMachine.setReportExceptions(false);

    try {
      stateMachine.initialize(ArmSequence.INVALID_TEST);
    } catch(StateMachineInitializationException ste) {
      ste.printStackTrace();
    }
    
    stateMachine.startSequence();
    assertEquals(Status.INVALID, stateMachine.getStatus(), "Status should be INVALID");
    assertEquals(stateMachine.getState(), ArmState.UNSAFE, "Status should be UNSAFE");
  }

  @Test 
  void interruptionAndRecovery() {
    try {
      // recreate the subsystem and state machine instances for this to test interruption
      mockSubsystem = new MockStateSubsystem();
      stateMachine = new ArmStateMachine(StateConstants.kArmStateMachineId, mockSubsystem);
      stateMachine.initialize(ArmSequence.SCORE_TEST);
    } catch(StateMachineInitializationException ste) {
      ste.printStackTrace();
    }

    assertEquals(StateConstants.kArmStateMachineId, stateMachine.getId(), "Arm machine ID doesn't match");
    assertEquals(Status.READY, stateMachine.getStatus(), "Status should be READY");

    // start sequence and let it run until wait condition
    stateMachine.startSequence();

    assertEquals(Status.PAUSED, stateMachine.getStatus(), "Status should be PAUSED");

    // now interrupt the sequence
    stateMachine.interruptSequence();

    assertEquals(Status.FINISHED, stateMachine.getStatus(), "Status should be FINISHED");
    assertTrue(mockSubsystem.wasInterrupted, "Subsystem doe not reflect interruption");

    // grab all processed steps
    var processedSteps = stateMachine.getProcessedSteps().toArray();
    assertEquals(5, processedSteps.length, "Wrong number of processed steps");
    // Step 1
    StateChange step = (StateChange)processedSteps[0];
    assertEquals(step.previousState, ArmState.EXTEND_MOVING, "[0] Previous state should be EXTEND_MOVING");
    assertEquals(step.newState, ArmState.EXTEND_PINGING, "[0] New state should be EXTEND_PINGING");
    assertEquals(step.request.input, ArmInput.EXTEND_PING, "[0] Request input should be EXTEND_PING");
    // Step 2
    step = (StateChange)processedSteps[1];
    assertEquals(step.previousState, ArmState.EXTEND_PINGING, "[1] Previous state should be EXTEND_PINGING");
    assertEquals(step.newState, ArmState.EXTENDED, "[1] New state should be EXTENDED");
    assertEquals(step.result.code, StateConstants.kSuccessCode, "[1] Result should indicate success");
    // Step 3
    step = (StateChange)processedSteps[2];
    assertEquals(step.previousState, ArmState.EXTENDED, "[2] Previous state should be EXTENDED");
    assertEquals(step.request.input, ArmInput.RELEASE, "[2] Request input should be RELEASE");
    assertEquals(step.newState, ArmState.INTERRUPTED, "[2] New state should be INTERRUPTED");
    assertTrue(step.interrupted, "[2] Should indicate interrupted");
    assertTrue(step.interruptedTimestamp > 0, "[2] Interrupted timestamp should be set");
    // Step 4
    step = (StateChange)processedSteps[3];
    assertEquals(step.previousState, ArmState.INTERRUPTED, "[3] Previous state should be INTERRUPTED");
    assertEquals(step.newState, ArmState.RECOVERING, "[3] New state should be RECOVERING");
    assertEquals(step.request.input, ArmInput.RECOVER, "[2] Request input should be RECOVER");
    // Step 5
    step = (StateChange)processedSteps[4];
    assertEquals(step.previousState, ArmState.RECOVERING, "[4] Previous state should be RECOVERING");
    assertEquals(step.newState, ArmState.RETRACTED, "[4] New state should be RETRACTED");
    assertEquals(step.result.code, StateConstants.kSuccessCode, "[4] Result should indicate success");
  }

  @Test 
  void failedAndRecovery() {
    try {
      // recreate the subsystem and state machine instances for this test to force a failure
      mockSubsystem = new MockStateSubsystem();
      mockSubsystem.shouldFail = true;
      stateMachine = new ArmStateMachine(StateConstants.kArmStateMachineId, mockSubsystem);
      stateMachine.initialize(ArmSequence.SCORE_TEST);
    } catch(StateMachineInitializationException ste) {
      ste.printStackTrace();
    }

    assertEquals(StateConstants.kArmStateMachineId, stateMachine.getId(), "Arm machine ID doesn't match");
    assertEquals(Status.READY, stateMachine.getStatus(), "Status should be READY");

    // start sequence and let it run until wait condition
    stateMachine.startSequence();

    assertEquals(Status.FINISHED, stateMachine.getStatus(), "Status should be FINISHED");

    // grab all processed steps
    var processedSteps = stateMachine.getProcessedSteps().toArray();
    assertEquals(4, processedSteps.length, "Wrong number of processed steps");
    // Step 1
    StateChange step = (StateChange)processedSteps[0];
    assertEquals(step.previousState, ArmState.EXTEND_INITIALIZED, "[0] Previous state should be EXTEND_INITIALIZED");
    assertEquals(step.newState, ArmState.EXTEND_MOVE_REQUESTED, "[0] New state should be EXTEND_MOVE_REQUESTED");
    assertEquals(step.request.input, ArmInput.EXTEND_MOVE, "[0] Request input should be EXTEND_MOVE");
    // Step 2
    step = (StateChange)processedSteps[1];
    assertEquals(step.previousState, ArmState.EXTEND_MOVE_REQUESTED, "[1] Previous state should be EXTEND_MOVE_REQUESTED");
    assertEquals(step.newState, ArmState.INTERRUPTED, "[1] New state should be INTERRUPTED");
    assertEquals(step.result.code, StateConstants.kGenericFailedCode, "[1] Result should indicate failure");
    // Step 3
    step = (StateChange)processedSteps[2];
    assertEquals(step.previousState, ArmState.INTERRUPTED, "[2] Previous state should be INTERRUPTED");
    assertEquals(step.newState, ArmState.RECOVERING, "[2] New state should be RECOVERING");
    assertEquals(step.request.input, ArmInput.RECOVER, "[2] Request input should be RECOVER");
    // Step 4
    step = (StateChange)processedSteps[3];
    assertEquals(step.previousState, ArmState.RECOVERING, "[3] Previous state should be RECOVERING");
    assertEquals(step.newState, ArmState.RETRACTED, "[3] New state should be RETRACTING");
    assertEquals(step.result.code, StateConstants.kSuccessCode, "[3] Result should indicate success");
  }
  */
}
