package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GamePiece;
import frc.robot.state.arm.ArmSequence;
import frc.robot.state.arm.ArmStateMachine;
import frc.robot.state.arm.ArmStateMachine.Status;
import frc.data.mp.*;

public class ArmScoreCommand extends CommandBase {
    private ArmStateMachine stateMachine;
    private ArmSequence sequence;
    private Joystick joystick;
    private int distalAxis;

    public ArmScoreCommand(ArmStateMachine stateMachine, ArmSequence sequence, Joystick joystick, int distalAxis) {
        this.stateMachine = stateMachine;
        this.sequence = sequence;
        this.joystick = joystick;
        this.distalAxis = distalAxis;
    }

    @Override
	public void initialize() {
        if(stateMachine.getStatus() == Status.RUNNING) {
            System.out.println("WARNING: cannot command a score when arm state is already running a movement");
            return;
        }

        stateMachine.setJoystickControl(joystick, distalAxis);

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

        if(sequence == ArmSequence.READ_KEYPAD) {
            stateMachine.scoreKeyedEntry();
        } else if(path != null) {
            stateMachine.score(path);
        }
	}

    @Override
    public void end(boolean interrupted) {
        stateMachine.buttonReleased();
    }
}
