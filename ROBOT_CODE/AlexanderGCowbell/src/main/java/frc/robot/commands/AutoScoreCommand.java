package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GamePiece;
import frc.robot.state.arm.ArmSequence;
import frc.robot.state.arm.ArmStateMachine;
import frc.robot.state.arm.ArmStateMachine.Status;
import frc.data.mp.*;

public class AutoScoreCommand extends CommandBase {
    private ArmStateMachine stateMachine;
    private ArmSequence sequence;
    private GamePiece pieceType;
    private boolean started = false;
    private boolean isFinished = false;

    public AutoScoreCommand(ArmStateMachine stateMachine, ArmSequence sequence, GamePiece pieceType) {
        this.stateMachine = stateMachine;
        this.sequence = sequence;
        this.pieceType = pieceType;
    }

    @Override
	public void initialize() {
        if(stateMachine.getStatus() == Status.RUNNING) {
            System.out.println("WARNING: autonomous cannot command a score when arm state is already running a movement");
            return;
        }

        stateMachine.setGamePiece(pieceType);
        stateMachine.setIsInAuto(true);

        ArmPath path = null;
        if(sequence == ArmSequence.SCORE_HIGH && stateMachine.getGamePiece() == GamePiece.CONE) {
            path = ScoreHighCone.getArmPath();
        } else if(sequence == ArmSequence.SCORE_HIGH && stateMachine.getGamePiece() == GamePiece.CUBE) {
            path = ScoreHighCube.getArmPath();
        } else if(sequence == ArmSequence.SCORE_MEDIUM && stateMachine.getGamePiece() == GamePiece.CONE) {
            path = ScoreMediumCone.getArmPath();
        } else if(sequence == ArmSequence.SCORE_MEDIUM && stateMachine.getGamePiece() == GamePiece.CUBE) {
            path = ScoreMediumCube.getArmPath();
        } else if(sequence == ArmSequence.SCORE_LOW && stateMachine.getGamePiece() == GamePiece.CONE) {
            path = ScoreLowCone.getArmPath();
        } else if(sequence == ArmSequence.SCORE_LOW && stateMachine.getGamePiece() == GamePiece.CUBE) {
            path = ScoreLowCube.getArmPath();
        }

        if(path != null) {
            stateMachine.score(path);
        } else {
            isFinished = true;
        }
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
