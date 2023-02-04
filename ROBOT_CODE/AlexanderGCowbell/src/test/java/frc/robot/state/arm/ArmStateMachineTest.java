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
import frc.robot.Constants.StateConstants.ResultCode;
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
    assertEquals(14, processedSteps.length, "Wrong number of processed steps");
    // Step 1
    StateChange step = (StateChange)processedSteps[0];
    assertEquals(step.previousState, ArmState.RETRACTED, "[0] Previous state should be RETRACTED");
    assertEquals(step.newState, ArmState.EXTEND_REQUESTED, "[0] New state should be EXTEND_REQUESTED");
    assertEquals(step.request.input, ArmInput.EXTEND_INIT, "[0] Request input should be EXTEND_INIT");
    // Step 2
    step = (StateChange)processedSteps[1];
    assertEquals(step.previousState, ArmState.EXTEND_REQUESTED, "[1] Previous state should be EXTEND_REQUESTED");
    assertEquals(step.newState, ArmState.EXTEND_INITIALIZED, "[1] New state should be EXTEND_INITIALIZED");
    assertEquals(step.result.code, ResultCode.SUCCESS, "[1] Result should indicate success");
    // Step 3
    step = (StateChange)processedSteps[2];
    assertEquals(step.previousState, ArmState.EXTEND_INITIALIZED, "[2] Previous state should be EXTEND_INITIALIZED");
    assertEquals(step.newState, ArmState.EXTEND_MOVE_REQUESTED, "[2] New state should be EXTEND_MOVE_REQUESTED");
    assertEquals(step.request.input, ArmInput.EXTEND_MOVE, "[2] Request input should be EXTEND_MOVE");
    // Step 4
    step = (StateChange)processedSteps[3];
    assertEquals(step.previousState, ArmState.EXTEND_MOVE_REQUESTED, "[3] Previous state should be EXTEND_MOVE_REQUESTED");
    assertEquals(step.newState, ArmState.EXTEND_MOVING, "[3] New state should be EXTEND_MOVE");
    assertEquals(step.result.code, ResultCode.SUCCESS, "[3] Result should indicate success");
    // Step 5
    step = (StateChange)processedSteps[4];
    assertEquals(step.previousState, ArmState.EXTEND_MOVING, "[4] Previous state should be EXTEND_MOVING");
    assertEquals(step.newState, ArmState.EXTEND_PINGING, "[4] New state should be EXTEND_PINGING");
    assertEquals(step.request.input, ArmInput.EXTEND_PING, "[4] Request input should be EXTEND_PING");
    // Step 6
    step = (StateChange)processedSteps[5];
    assertEquals(step.previousState, ArmState.EXTEND_PINGING, "[5] Previous state should be EXTEND_PINGING");
    assertEquals(step.newState, ArmState.EXTENDED, "[5] New state should be EXTENDED");
    assertEquals(step.result.code, ResultCode.SUCCESS, "[5] Result should indicate success");
    // Step 7
    step = (StateChange)processedSteps[6];
    assertEquals(step.previousState, ArmState.EXTENDED, "[6] Previous state should be EXTENDED");
    assertEquals(step.newState, ArmState.RELEASING, "[6] New state should be RELEASING");
    assertEquals(step.request.input, ArmInput.RELEASE, "[6] Request input should be RELEASE");
    // Step 8
    step = (StateChange)processedSteps[7];
    assertEquals(step.previousState, ArmState.RELEASING, "[7] Previous state should be RELEASING");
    assertEquals(step.newState, ArmState.RELEASED, "[7] New state should be RELEASED");
    assertEquals(step.result.code, ResultCode.SUCCESS, "[7] Result should indicate success");
    // Step 9
    step = (StateChange)processedSteps[8];
    assertEquals(step.previousState, ArmState.RELEASED, "[8] Previous state should be RELEASED");
    assertEquals(step.newState, ArmState.RETRACT_REQUESTED, "[8] New state should be RETRACT_REQUESTED");
    assertEquals(step.request.input, ArmInput.RETRACT_INIT, "[8] Request input should be RETRACT_INIT");
    // Step 10
    step = (StateChange)processedSteps[9];
    assertEquals(step.previousState, ArmState.RETRACT_REQUESTED, "[9] Previous state should be RETRACT_REQUESTED");
    assertEquals(step.newState, ArmState.RETRACT_INITIALIZED, "[9] New state should be RETRACT_INITIALIZED");
    assertEquals(step.result.code, ResultCode.SUCCESS, "[9] Result should indicate success");
    // Step 11
    step = (StateChange)processedSteps[10];
    assertEquals(step.previousState, ArmState.RETRACT_INITIALIZED, "[10] Previous state should be RETRACT_INITIALIZED");
    assertEquals(step.newState, ArmState.RETRACT_MOVE_REQUESTED, "[10] New state should be RETRACT_MOVE_REQUESTED");
    assertEquals(step.request.input, ArmInput.RETRACT_MOVE, "[10] Request input should be RETRACT_MOVE");
    // Step 12
    step = (StateChange)processedSteps[11];
    assertEquals(step.previousState, ArmState.RETRACT_MOVE_REQUESTED, "[11] Previous state should be RETRACT_MOVE_REQUESTED");
    assertEquals(step.newState, ArmState.RETRACT_MOVING, "[11] New state should be RETRACT_MOVING");
    assertEquals(step.result.code, ResultCode.SUCCESS, "[11] Result should indicate success");
    // Step 13
    step = (StateChange)processedSteps[12];
    assertEquals(step.previousState, ArmState.RETRACT_MOVING, "[12] Previous state should be RETRACT_MOVING");
    assertEquals(step.newState, ArmState.RETRACT_PINGING, "[12] New state should be RETRACT_PINGING");
    assertEquals(step.request.input, ArmInput.RETRACT_PING, "[12] Request input should be RETRACT_PING");
    // Step 14
    step = (StateChange)processedSteps[13];
    assertEquals(step.previousState, ArmState.RETRACT_PINGING, "[13] Previous state should be RETRACT_PINGING");
    assertEquals(step.newState, ArmState.RETRACTED, "[13] New state should be RETRACTED");
    assertEquals(step.result.code, ResultCode.SUCCESS, "[13] Result should indicate success");
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
    assertEquals(14, processedSteps.length, "Wrong number of processed steps");
    // Step 1
    StateChange step = (StateChange)processedSteps[0];
    assertEquals(step.previousState, ArmState.RETRACTED, "[0] Previous state should be RETRACTED");
    assertEquals(step.newState, ArmState.EXTEND_REQUESTED, "[0] New state should be EXTEND_REQUESTED");
    assertEquals(step.request.input, ArmInput.EXTEND_INIT, "[0] Request input should be EXTEND_INIT");
    // Step 2
    step = (StateChange)processedSteps[1];
    assertEquals(step.previousState, ArmState.EXTEND_REQUESTED, "[1] Previous state should be EXTEND_REQUESTED");
    assertEquals(step.newState, ArmState.EXTEND_INITIALIZED, "[1] New state should be EXTEND_INITIALIZED");
    assertEquals(step.result.code, ResultCode.SUCCESS, "[1] Result should indicate success");
    // Step 3
    step = (StateChange)processedSteps[2];
    assertEquals(step.previousState, ArmState.EXTEND_INITIALIZED, "[2] Previous state should be EXTEND_INITIALIZED");
    assertEquals(step.newState, ArmState.EXTEND_MOVE_REQUESTED, "[2] New state should be EXTEND_MOVE_REQUESTED");
    assertEquals(step.request.input, ArmInput.EXTEND_MOVE, "[2] Request input should be EXTEND_MOVE");
    // Step 4
    step = (StateChange)processedSteps[3];
    assertEquals(step.previousState, ArmState.EXTEND_MOVE_REQUESTED, "[3] Previous state should be EXTEND_MOVE_REQUESTED");
    assertEquals(step.newState, ArmState.EXTEND_MOVING, "[3] New state should be EXTEND_MOVE");
    assertEquals(step.result.code, ResultCode.SUCCESS, "[3] Result should indicate success");
    // Step 5
    step = (StateChange)processedSteps[4];
    assertEquals(step.previousState, ArmState.EXTEND_MOVING, "[4] Previous state should be EXTEND_MOVING");
    assertEquals(step.newState, ArmState.EXTEND_PINGING, "[4] New state should be EXTEND_PINGING");
    assertEquals(step.request.input, ArmInput.EXTEND_PING, "[4] Request input should be EXTEND_PING");
    // Step 6
    step = (StateChange)processedSteps[5];
    assertEquals(step.previousState, ArmState.EXTEND_PINGING, "[5] Previous state should be EXTEND_PINGING");
    assertEquals(step.newState, ArmState.EXTENDED, "[5] New state should be EXTENDED");
    assertEquals(step.result.code, ResultCode.SUCCESS, "[5] Result should indicate success");
    // Step 7
    step = (StateChange)processedSteps[6];
    assertEquals(step.previousState, ArmState.EXTENDED, "[6] Previous state should be EXTENDED");
    assertEquals(step.newState, ArmState.RETRIEVING, "[6] New state should be RETRIEVING");
    assertEquals(step.request.input, ArmInput.INTAKE, "[6] Request input should be INTAKE");
    // Step 8
    step = (StateChange)processedSteps[7];
    assertEquals(step.previousState, ArmState.RETRIEVING, "[7] Previous state should be RETRIEVING");
    assertEquals(step.newState, ArmState.RETRIEVED, "[7] New state should be RETRIEVED");
    assertEquals(step.result.code, ResultCode.SUCCESS, "[5] Result should indicate success");
    // Step 9
    step = (StateChange)processedSteps[8];
    assertEquals(step.previousState, ArmState.RETRIEVED, "[8] Previous state should be RETRIEVED");
    assertEquals(step.newState, ArmState.RETRACT_REQUESTED, "[8] New state should be RETRACT_REQUESTED");
    assertEquals(step.request.input, ArmInput.RETRACT_INIT, "[8] Request input should be RETRACT_INIT");
    // Step 10
    step = (StateChange)processedSteps[9];
    assertEquals(step.previousState, ArmState.RETRACT_REQUESTED, "[9] Previous state should be RETRACT_REQUESTED");
    assertEquals(step.newState, ArmState.RETRACT_INITIALIZED, "[9] New state should be RETRACT_INITIALIZED");
    assertEquals(step.result.code, ResultCode.SUCCESS, "[9] Result should indicate success");
    // Step 11
    step = (StateChange)processedSteps[10];
    assertEquals(step.previousState, ArmState.RETRACT_INITIALIZED, "[10] Previous state should be RETRACT_INITIALIZED");
    assertEquals(step.newState, ArmState.RETRACT_MOVE_REQUESTED, "[10] New state should be RETRACT_MOVE_REQUESTED");
    assertEquals(step.request.input, ArmInput.RETRACT_MOVE, "[10] Request input should be RETRACT_MOVE");
    // Step 12
    step = (StateChange)processedSteps[11];
    assertEquals(step.previousState, ArmState.RETRACT_MOVE_REQUESTED, "[11] Previous state should be RETRACT_MOVE_REQUESTED");
    assertEquals(step.newState, ArmState.RETRACT_MOVING, "[11] New state should be RETRACT_MOVING");
    assertEquals(step.result.code, ResultCode.SUCCESS, "[11] Result should indicate success");
    // Step 13
    step = (StateChange)processedSteps[12];
    assertEquals(step.previousState, ArmState.RETRACT_MOVING, "[12] Previous state should be RETRACT_MOVING");
    assertEquals(step.newState, ArmState.RETRACT_PINGING, "[12] New state should be RETRACT_PINGING");
    assertEquals(step.request.input, ArmInput.RETRACT_PING, "[12] Request input should be RETRACT_PING");
    // Step 14
    step = (StateChange)processedSteps[13];
    assertEquals(step.previousState, ArmState.RETRACT_PINGING, "[13] Previous state should be RETRACT_PINGING");
    assertEquals(step.newState, ArmState.RETRACTED, "[13] New state should be RETRACTED");
    assertEquals(step.result.code, ResultCode.SUCCESS, "[13] Result should indicate success");
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
    assertEquals(9, processedSteps.length, "Wrong number of processed steps");
    // Step 1
    StateChange step = (StateChange)processedSteps[0];
    assertEquals(step.previousState, ArmState.RETRACTED, "[0] Previous state should be RETRACTED");
    assertEquals(step.newState, ArmState.EXTEND_REQUESTED, "[0] New state should be EXTEND_REQUESTED");
    assertEquals(step.request.input, ArmInput.EXTEND_INIT, "[0] Request input should be EXTEND_INIT");
    // Step 2
    step = (StateChange)processedSteps[1];
    assertEquals(step.previousState, ArmState.EXTEND_REQUESTED, "[1] Previous state should be EXTEND_REQUESTED");
    assertEquals(step.newState, ArmState.EXTEND_INITIALIZED, "[1] New state should be EXTEND_INITIALIZED");
    assertEquals(step.result.code, ResultCode.SUCCESS, "[1] Result should indicate success");
    // Step 3
    step = (StateChange)processedSteps[2];
    assertEquals(step.previousState, ArmState.EXTEND_INITIALIZED, "[2] Previous state should be EXTEND_INITIALIZED");
    assertEquals(step.newState, ArmState.EXTEND_MOVE_REQUESTED, "[2] New state should be EXTEND_REQUESTED");
    assertEquals(step.request.input, ArmInput.EXTEND_MOVE, "[2] Request input should be EXTEND_MOVE");
    // Step 4
    step = (StateChange)processedSteps[3];
    assertEquals(step.previousState, ArmState.EXTEND_MOVE_REQUESTED, "[3] Previous state should be EXTEND_MOVE_REQUESTED");
    assertEquals(step.newState, ArmState.EXTEND_MOVING, "[3] New state should be EXTEND_MOVING");
    assertEquals(step.result.code, ResultCode.SUCCESS, "[3] Result should indicate success");
    // Step 5
    step = (StateChange)processedSteps[4];
    assertEquals(step.previousState, ArmState.EXTEND_MOVING, "[4] Previous state should be EXTEND_MOVING");
    assertEquals(step.newState, ArmState.EXTEND_PINGING, "[4] New state should be EXTEND_PINGING");
    assertEquals(step.request.input, ArmInput.EXTEND_PING, "[4] Request input should be EXTEND_PING");
    // Step 6
    step = (StateChange)processedSteps[5];
    assertEquals(step.previousState, ArmState.EXTEND_PINGING, "[5] Previous state should be EXTEND_PINGING");
    assertEquals(step.newState, ArmState.EXTENDED, "[5] New state should be EXTENDED");
    assertEquals(step.result.code, ResultCode.SUCCESS, "[5] Result should indicate success");
    // Step 7
    step = (StateChange)processedSteps[6];
    assertEquals(step.previousState, ArmState.EXTENDED, "[6] Previous state should be EXTENDED");
    assertEquals(step.request.input, ArmInput.RELEASE, "[6] Request input should be RELEASE");
    assertEquals(step.newState, ArmState.INTERRUPTED, "[6] New state should be INTERRUPTED");
    assertTrue(step.interrupted, "[6] Should indicate interrupted");
    assertTrue(step.interruptedTimestamp > 0, "[6] Interrupted timestamp should be set");
    // Step 8
    step = (StateChange)processedSteps[7];
    assertEquals(step.previousState, ArmState.INTERRUPTED, "[7] Previous state should be INTERRUPTED");
    assertEquals(step.newState, ArmState.RECOVERING, "[7] New state should be RECOVERING");
    assertEquals(step.request.input, ArmInput.RECOVER, "[7] Request input should be RECOVER");
    // Step 9
    step = (StateChange)processedSteps[8];
    assertEquals(step.previousState, ArmState.RECOVERING, "[8] Previous state should be RECOVERING");
    assertEquals(step.newState, ArmState.RETRACTED, "[8] New state should be RETRACTED");
    assertEquals(step.result.code, ResultCode.SUCCESS, "[8] Result should indicate success");
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
    assertEquals(step.previousState, ArmState.RETRACTED, "[0] Previous state should be RETRACTED");
    assertEquals(step.newState, ArmState.EXTEND_REQUESTED, "[0] New state should be EXTEND_REQUESTED");
    assertEquals(step.request.input, ArmInput.EXTEND_INIT, "[0] Request input should be EXTEND_INIT");
    // Step 2
    step = (StateChange)processedSteps[1];
    assertEquals(step.previousState, ArmState.EXTEND_REQUESTED, "[1] Previous state should be EXTEND_REQUESTED");
    assertEquals(step.newState, ArmState.INTERRUPTED, "[1] New state should be INTERRUPTED");
    assertEquals(step.result.code, ResultCode.FAILED, "[1] Result should indicate failure");
    // Step 3
    step = (StateChange)processedSteps[2];
    assertEquals(step.previousState, ArmState.INTERRUPTED, "[2] Previous state should be INTERRUPTED");
    assertEquals(step.newState, ArmState.RECOVERING, "[2] New state should be RECOVERING");
    assertEquals(step.request.input, ArmInput.RECOVER, "[2] Request input should be RECOVER");
    // Step 4
    step = (StateChange)processedSteps[3];
    assertEquals(step.previousState, ArmState.RECOVERING, "[3] Previous state should be RECOVERING");
    assertEquals(step.newState, ArmState.RETRACTED, "[3] New state should be RETRACTED");
    assertEquals(step.result.code, ResultCode.SUCCESS, "[3] Result should indicate success");
  }
}
