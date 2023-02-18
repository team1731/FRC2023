package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.state.arm.ArmSequence;
import frc.robot.state.arm.ArmState;
import frc.robot.state.arm.ArmStateMachine;
import frc.robot.state.arm.ArmStateMachine.Status;
import frc.data.mp.*;

public class AutoScoreCommand extends CommandBase {
    private ArmStateMachine stateMachine;
    private ArmSequence sequence;
    private boolean started = false;
    private boolean isFinished = false;

    public AutoScoreCommand(ArmStateMachine stateMachine, ArmSequence sequence) {
        this.stateMachine = stateMachine;
        this.sequence = sequence;
    }

    @Override
	public void initialize() {
        ArmPath path = null;
        if(sequence == ArmSequence.SCORE_HIGH) {
            path = ScoreHigh.getArmPath();
        } else if(sequence == ArmSequence.SCORE_MEDIUM) {
            path = ScoreMedium.getArmPath();
        } else {
            path = ScoreLow.getArmPath();
        }

        stateMachine.score(path);
	}

    @Override
    public void execute() {
        if(!started && stateMachine.getStatus() == Status.RUNNING) {
            started = true;
        } else if(started && stateMachine.getStatus() == Status.READY) {
            // has returned to a ready state, we are done
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
