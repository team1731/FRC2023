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
    stateMachine = (ArmStateMachine)mockSubsystem.getStateMachine();
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
    assertEquals(10, processedSteps.length, "Wrong number of processed steps");
    // Step 1
    StateChange step = (StateChange)processedSteps[0];
    assertEquals(step.previousState, ArmState.RETRACTED, "[0] Previous state should be RETRACTED");
    assertEquals(step.newState, ArmState.EXTEND_REQUESTED, "[0] New state should be EXTEND_REQUESTED");
    assertEquals(step.request.input, ArmInput.EXTEND, "[0] Request input should be EXTEND");
    // Step 2
    step = (StateChange)processedSteps[1];
    assertEquals(step.previousState, ArmState.EXTEND_REQUESTED, "[1] Previous state should be EXTEND_REQUESTED");
    assertEquals(step.newState, ArmState.EXTENDING, "[1] New state should be EXTENDING");
    assertEquals(step.result.code, ResultCode.SUCCESS, "[1] Result should indicate success");
    // Step 3
    step = (StateChange)processedSteps[2];
    assertEquals(step.previousState, ArmState.EXTENDING, "[2] Previous state should be EXTENDING");
    assertEquals(step.newState, ArmState.EXTEND_PINGING, "[2] New state should be EXTEND_PINGING");
    assertEquals(step.request.input, ArmInput.EXTEND_PING, "[2] Request input should be EXTEND_PING");
    // Step 4
    step = (StateChange)processedSteps[3];
    assertEquals(step.previousState, ArmState.EXTEND_PINGING, "[3] Previous state should be EXTEND_PINGING");
    assertEquals(step.newState, ArmState.EXTENDED, "[3] New state should be EXTENDED");
    assertEquals(step.result.code, ResultCode.SUCCESS, "[3] Result should indicate success");
    // Step 5
    step = (StateChange)processedSteps[4];
    assertEquals(step.previousState, ArmState.EXTENDED, "[4] Previous state should be EXTENDED");
    assertEquals(step.newState, ArmState.RELEASING, "[4] New state should be RELEASING");
    assertEquals(step.request.input, ArmInput.RELEASE, "[4] Request input should be RELEASE");
    // Step 6
    step = (StateChange)processedSteps[5];
    assertEquals(step.previousState, ArmState.RELEASING, "[5] Previous state should be RELEASING");
    assertEquals(step.newState, ArmState.RELEASED, "[5] New state should be RELEASED");
    assertEquals(step.result.code, ResultCode.SUCCESS, "[5] Result should indicate success");
    // Step 7
    step = (StateChange)processedSteps[6];
    assertEquals(step.previousState, ArmState.RELEASED, "[6] Previous state should be RELEASED");
    assertEquals(step.newState, ArmState.RETRACT_REQUESTED, "[6] New state should be RETRACT_REQUESTED");
    assertEquals(step.request.input, ArmInput.RETRACT, "[6] Request input should be RETRACT");
    // Step 8
    step = (StateChange)processedSteps[7];
    assertEquals(step.previousState, ArmState.RETRACT_REQUESTED, "[7] Previous state should be RETRACT_REQUESTED");
    assertEquals(step.newState, ArmState.RETRACTING, "[7] New state should be RETRACTING");
    assertEquals(step.result.code, ResultCode.SUCCESS, "[7] Result should indicate success");
    // Step 9
    step = (StateChange)processedSteps[8];
    assertEquals(step.previousState, ArmState.RETRACTING, "[8] Previous state should be RETRACTING");
    assertEquals(step.newState, ArmState.RETRACT_PINGING, "[8] New state should be RETRACT_PINGING");
    assertEquals(step.request.input, ArmInput.RETRACT_PING, "[8] Request input should be RETRACT_PING");
    // Step 10
    step = (StateChange)processedSteps[9];
    assertEquals(step.previousState, ArmState.RETRACT_PINGING, "[9] Previous state should be RETRACT_PINGING");
    assertEquals(step.newState, ArmState.RETRACTED, "[9] New state should be RETRACTED");
    assertEquals(step.result.code, ResultCode.SUCCESS, "[9] Result should indicate success");
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
    assertEquals(10, processedSteps.length, "Wrong number of processed steps");
   // Step 1
   StateChange step = (StateChange)processedSteps[0];
   assertEquals(step.previousState, ArmState.RETRACTED, "[0] Previous state should be RETRACTED");
   assertEquals(step.newState, ArmState.EXTEND_REQUESTED, "[0] New state should be EXTEND_REQUESTED");
   assertEquals(step.request.input, ArmInput.EXTEND, "[0] Request input should be EXTEND");
   // Step 2
   step = (StateChange)processedSteps[1];
   assertEquals(step.previousState, ArmState.EXTEND_REQUESTED, "[1] Previous state should be EXTEND_REQUESTED");
   assertEquals(step.newState, ArmState.EXTENDING, "[1] New state should be EXTENDING");
   assertEquals(step.result.code, ResultCode.SUCCESS, "[1] Result should indicate success");
   // Step 3
   step = (StateChange)processedSteps[2];
   assertEquals(step.previousState, ArmState.EXTENDING, "[2] Previous state should be EXTENDING");
   assertEquals(step.newState, ArmState.EXTEND_PINGING, "[2] New state should be EXTEND_PINGING");
   assertEquals(step.request.input, ArmInput.EXTEND_PING, "[2] Request input should be EXTEND_PING");
   // Step 4
   step = (StateChange)processedSteps[3];
   assertEquals(step.previousState, ArmState.EXTEND_PINGING, "[3] Previous state should be EXTEND_PINGING");
   assertEquals(step.newState, ArmState.EXTENDED, "[3] New state should be EXTENDED");
   assertEquals(step.result.code, ResultCode.SUCCESS, "[3] Result should indicate success");
   // Step 5
   step = (StateChange)processedSteps[4];
   assertEquals(step.previousState, ArmState.EXTENDED, "[4] Previous state should be EXTENDED");
   assertEquals(step.newState, ArmState.RETRIEVING, "[4] New state should be RETRIEVING");
   assertEquals(step.request.input, ArmInput.RETRIEVE, "[4] Request input should be RETRIEVE");
   // Step 6
   step = (StateChange)processedSteps[5];
   assertEquals(step.previousState, ArmState.RETRIEVING, "[5] Previous state should be RETRIEVING");
   assertEquals(step.newState, ArmState.RETRIEVED, "[5] New state should be RETRIEVED");
   assertEquals(step.result.code, ResultCode.SUCCESS, "[5] Result should indicate success");
   // Step 7
   step = (StateChange)processedSteps[6];
   assertEquals(step.previousState, ArmState.RETRIEVED, "[6] Previous state should be RETRIEVED");
   assertEquals(step.newState, ArmState.RETRACT_REQUESTED, "[6] New state should be RETRACT_REQUESTED");
   assertEquals(step.request.input, ArmInput.RETRACT, "[6] Request input should be RETRACT");
   // Step 8
   step = (StateChange)processedSteps[7];
   assertEquals(step.previousState, ArmState.RETRACT_REQUESTED, "[7] Previous state should be RETRACT_REQUESTED");
   assertEquals(step.newState, ArmState.RETRACTING, "[7] New state should be RETRACTING");
   assertEquals(step.result.code, ResultCode.SUCCESS, "[7] Result should indicate success");
   // Step 9
   step = (StateChange)processedSteps[8];
   assertEquals(step.previousState, ArmState.RETRACTING, "[8] Previous state should be RETRACTING");
   assertEquals(step.newState, ArmState.RETRACT_PINGING, "[8] New state should be RETRACT_PINGING");
   assertEquals(step.request.input, ArmInput.RETRACT_PING, "[8] Request input should be RETRACT_PING");
   // Step 10
   step = (StateChange)processedSteps[9];
   assertEquals(step.previousState, ArmState.RETRACT_PINGING, "[9] Previous state should be RETRACT_PINGING");
   assertEquals(step.newState, ArmState.RETRACTED, "[9] New state should be RETRACTED");
   assertEquals(step.result.code, ResultCode.SUCCESS, "[9] Result should indicate success");
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
      stateMachine = (ArmStateMachine)mockSubsystem.getStateMachine();
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
    assertEquals(7, processedSteps.length, "Wrong number of processed steps");
    // Step 1
    StateChange step = (StateChange)processedSteps[0];
    assertEquals(step.previousState, ArmState.RETRACTED, "[0] Previous state should be RETRACTED");
    assertEquals(step.newState, ArmState.EXTEND_REQUESTED, "[0] New state should be EXTEND_REQUESTED");
    assertEquals(step.request.input, ArmInput.EXTEND, "[0] Request input should be EXTEND");
    // Step 2
    step = (StateChange)processedSteps[1];
    assertEquals(step.previousState, ArmState.EXTEND_REQUESTED, "[1] Previous state should be EXTEND_REQUESTED");
    assertEquals(step.newState, ArmState.EXTENDING, "[1] New state should be EXTENDING");
    assertEquals(step.result.code, ResultCode.SUCCESS, "[1] Result should indicate success");
    // Step 3
    step = (StateChange)processedSteps[2];
    assertEquals(step.previousState, ArmState.EXTENDING, "[2] Previous state should be EXTENDING");
    assertEquals(step.newState, ArmState.EXTEND_PINGING, "[2] New state should be EXTEND_PINGING");
    assertEquals(step.request.input, ArmInput.EXTEND_PING, "[2] Request input should be EXTEND_PING");
    // Step 4
    step = (StateChange)processedSteps[3];
    assertEquals(step.previousState, ArmState.EXTEND_PINGING, "[3] Previous state should be EXTEND_PINGING");
    assertEquals(step.newState, ArmState.EXTENDED, "[3] New state should be EXTENDED");
    assertEquals(step.result.code, ResultCode.SUCCESS, "[3] Result should indicate success");
    // Step 5
    step = (StateChange)processedSteps[4];
    assertEquals(step.previousState, ArmState.EXTENDED, "[4] Previous state should be EXTENDED");
    assertEquals(step.request.input, ArmInput.RELEASE, "[4] Request input should be RELEASE");
    assertEquals(step.newState, ArmState.INTERRUPTED, "[4] New state should be INTERRUPTED");
    assertTrue(step.interrupted, "[4] Should indicate interrupted");
    assertTrue(step.interruptedTimestamp > 0, "[4] Interrupted timestamp should be set");
    // Step 6
    step = (StateChange)processedSteps[5];
    assertEquals(step.previousState, ArmState.INTERRUPTED, "[5] Previous state should be INTERRUPTED");
    assertEquals(step.newState, ArmState.RECOVERING, "[5] New state should be RECOVERING");
    assertEquals(step.request.input, ArmInput.RECOVER, "[5] Request input should be RECOVER");
    // Step 7
    step = (StateChange)processedSteps[6];
    assertEquals(step.previousState, ArmState.RECOVERING, "[6] Previous state should be RECOVERING");
    assertEquals(step.newState, ArmState.RETRACTED, "[6] New state should be RETRACTED");
    assertEquals(step.result.code, ResultCode.SUCCESS, "[6] Result should indicate success");
  }

  @Test 
  void failedAndRecovery() {
    try {
      // recreate the subsystem and state machine instances for this test to force a failure
      mockSubsystem = new MockStateSubsystem();
      mockSubsystem.shouldFail = true;
      stateMachine = (ArmStateMachine)mockSubsystem.getStateMachine();
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
    assertEquals(step.request.input, ArmInput.EXTEND, "[0] Request input should be EXTEND");
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