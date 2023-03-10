package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GamePiece;
import frc.robot.state.arm.ArmSequence;
import frc.robot.state.arm.ArmStateMachine;
import frc.data.mp.*;

public class ArmScoreCommand extends CommandBase {
    private ArmStateMachine stateMachine;
    private ArmSequence sequence;
    private Joystick joystick;
    private int distalAxis;
    private double queuedTime;
    private boolean isFinished = false;

    public ArmScoreCommand(ArmStateMachine stateMachine, ArmSequence sequence, Joystick joystick, int distalAxis) {
        this.stateMachine = stateMachine;
        this.sequence = sequence;
        this.joystick = joystick;
        this.distalAxis = distalAxis;
    }

    @Override
	public void initialize() {
        isFinished = false;
        
        // Queued time used to distinguish running path from queued path if both are present
        queuedTime = Timer.getFPGATimestamp();

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

        stateMachine.addJoystickControl(joystick, distalAxis, false);

        if(sequence == ArmSequence.READ_OPERATOR_ENTRY) {
            stateMachine.scoreOperatorEntry(queuedTime);
        } else if(path != null) {
            stateMachine.score(path, queuedTime);
        }
	}

    @Override
    public void end(boolean interrupted) {
        stateMachine.buttonReleased(queuedTime);
        isFinished = true;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
