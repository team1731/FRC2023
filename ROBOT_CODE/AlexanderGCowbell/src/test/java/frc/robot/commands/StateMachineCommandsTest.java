package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import frc.robot.state.*;
import frc.robot.state.StateMachine.Status;
import frc.robot.state.arm.ArmStateMachine;
import frc.robot.state.arm.ArmWait;
import frc.robot.state.arm.ArmSequence;
import frc.robot.testsupport.mock.MockStateSubsystem;

public class StateMachineCommandsTest {
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
    void successfulScoreCommand() {
        StateMachineCommand stateMachineCommand = new StateMachineCommand(stateMachine, ArmSequence.SCORE_TEST);

        // initialize command, will kick off state machine sequence
        stateMachineCommand.initialize();

        // this sequence has a wait condition, so it should be waiting
        assertEquals(Status.PAUSED, stateMachine.getStatus(), "Status should be PAUSED");

        // simulate resolving wait condition from a parallel process
        stateMachine.resolveWaitCondition(ArmWait.UNTIL_LINED_UP_FOR_SCORING);

        // call execute, its called repeatedly while command is scheduled, every ~20ms
        // calls state machine periodic() in turn, which will see the wait condition is resolved and restart
        stateMachineCommand.execute();

        // sequence should be finished
        assertEquals(Status.FINISHED, stateMachine.getStatus(), "Status should be FINISHED");
        assertFalse(stateMachineCommand.isFinished(), "Command should still be running");

        // call execute one more time, should see the state machine has completed and mark itself finished
        stateMachineCommand.execute();
        assertTrue(stateMachineCommand.isFinished(), "Command should be finished");
    }

    @Test 
    void canceledCommand() {
        StateMachineCommand stateMachineCommand = new StateMachineCommand(stateMachine, ArmSequence.SCORE_TEST);

        // initialize command, will kick off state machine sequence
        stateMachineCommand.initialize();

        // this sequence has a wait condition, so it should be waiting
        assertEquals(Status.PAUSED, stateMachine.getStatus(), "Status should be PAUSED");

        // simulate ending command with interruption
        stateMachineCommand.end(true);

        // sequence should be finished
        assertEquals(Status.FINISHED, stateMachine.getStatus(), "Status should be FINISHED");

        var processedSteps = stateMachine.getProcessedSteps().toArray();
        StateChange step = (StateChange)processedSteps[2];
        // step 3 should show the interruption
        //assertTrue(step.interrupted, "Processed steps should indicate interrupted");
        //assertTrue(step.interruptedTimestamp > 0, "Interrupted timestamp should be set");
    }

    @Test 
    void handlesInitializationFailure() {
        StateMachineCommand stateMachineCommand = new StateMachineCommand(stateMachine, ArmSequence.INVALID_TEST);
        stateMachineCommand.setReportExceptions(false);
        stateMachine.setReportExceptions(false);

        // initialize command, will kick off state machine sequence
        stateMachineCommand.initialize();

        // state machine should show invalid status
        assertEquals(Status.INVALID, stateMachine.getStatus(), "Status should be INVALID");

        // call execute, called repeatedly while command is scheduled, every ~20ms
        // calls state machine periodic() in turn, which will see the invalid status and put itself in finished state
        stateMachineCommand.execute();

        // command should be finished
        assertTrue(stateMachineCommand.isFinished(), "Command should be finished");
    }
}
