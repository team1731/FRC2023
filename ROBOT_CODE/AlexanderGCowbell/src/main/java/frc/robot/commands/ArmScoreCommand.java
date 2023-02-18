package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.state.arm.ArmSequence;
import frc.robot.state.arm.ArmStateMachine;
import frc.robot.state.arm.ArmStateMachine.Status;
import frc.data.mp.*;

public class ArmScoreCommand extends CommandBase {
    private ArmStateMachine stateMachine;
    private ArmSequence sequence;

    public ArmScoreCommand(ArmStateMachine stateMachine, ArmSequence sequence) {
        this.stateMachine = stateMachine;
        if(sequence == ArmSequence.READ_KEYPAD) {
            this.sequence = ArmSequence.valueForCode("REPLACE_WITH_KEYPAD_CODE");
        } else {
            this.sequence = sequence;
        }
    }

    @Override
	public void initialize() {
        if(stateMachine.getStatus() == Status.RUNNING) {
            System.out.println("WARNING: cannot command a score when arm state is already running a movement");
            return;
        }

        ArmPath path = null;
        if(sequence == ArmSequence.SCORE_HIGH) {
            path = ScoreHigh.getArmPath();
        } else if(sequence == ArmSequence.SCORE_MEDIUM) {
            path = ScoreMedium.getArmPath();
        } else if(sequence == ArmSequence.SCORE_LOW) {
            path = ScoreLow.getArmPath();
        }

        if(path != null) {
            stateMachine.score(path);
        }
	}

    @Override
    public void end(boolean interrupted) {
        stateMachine.buttonReleased();
    }
}
