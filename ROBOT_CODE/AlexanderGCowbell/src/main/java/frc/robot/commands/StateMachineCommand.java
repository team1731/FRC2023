package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.state.*;
import frc.robot.state.StateMachine.Status;


public class StateMachineCommand extends CommandBase {
    private StateMachine stateMachine;
    private StateSequence sequence;
    private boolean finished;
    private boolean reportExceptions = true; // should only be false during unit testing

    public StateMachineCommand(StateMachine stateMachine, StateSequence sequence) {
        this.stateMachine = stateMachine;
        this.sequence = sequence;
    }

    // used for unit testing only to prevent intentionally intiated exceptions from printing out
    public void setReportExceptions(boolean reportExceptions) {
        this.reportExceptions = reportExceptions;
    }

    @Override
	public void initialize() {
        try {
            stateMachine.initialize(sequence);
            stateMachine.startSequence();
        } catch(StateMachineInitializationException ste) {
            if(reportExceptions) {
                DriverStation.reportError(ste.getMessage(), ste.getStackTrace());
            }
            finished = true;
        }
	}

    @Override
	public void execute() {
        var smStatus = stateMachine.getStatus();
		if(smStatus == Status.INTERRUPTED || smStatus == Status.FINISHED || smStatus == Status.INVALID) {
            finished = true;
        }
	}

    @Override
	public void end(boolean interrupted) {
        if(interrupted) {
            stateMachine.interruptSequence();
        }
        finished = true;
	}

    @Override
    public boolean isFinished() {
        return finished;
    }
}
