package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ArmStateConstants;
import frc.robot.Constants.GamePiece;
import frc.robot.state.arm.ArmSequence;
import frc.robot.state.arm.ArmStateMachine;
import frc.robot.state.arm.ArmStateMachine.Status;
import frc.data.mp.*;

public class AutoPickupCommand extends CommandBase {
    private ArmStateMachine stateMachine;
    private ArmSequence sequence;
    private GamePiece pieceType;
    private boolean started = false;
    private boolean isFinished = false;
    private double queuedTime;

    public AutoPickupCommand(ArmStateMachine stateMachine, ArmSequence sequence, GamePiece pieceType) {
        this.stateMachine = stateMachine;
        this.sequence = sequence;
        this.pieceType = pieceType;
        
    }

    @Override
	public void initialize() {
        System.out.println("Starting the pickup..........................................................*****************************88");
        stateMachine.setGamePiece(pieceType);

        // Queued time used to distinguish running path from queued path if both are present
        queuedTime = Timer.getFPGATimestamp();

        ArmPath path = null;
        if(sequence == ArmSequence.PICKUP_HIGH && stateMachine.getGamePiece() == GamePiece.CONE) {
            path = PickupHighCone.getArmPath();
        } else if(sequence == ArmSequence.PICKUP_HIGH && stateMachine.getGamePiece() == GamePiece.CUBE) {
            path = PickupHighCube.getArmPath();
        } else if(sequence == ArmSequence.PICKUP_LOW && stateMachine.getGamePiece() == GamePiece.CONE) {
            path = PickupLowCone.getArmPath();
        } else if(sequence == ArmSequence.PICKUP_LOW && stateMachine.getGamePiece() == GamePiece.CUBE) {
            path = PickupLowCube.getArmPath();
        }

        if(path != null) {
            stateMachine.pickup(path, queuedTime);
        } else {
            isFinished = true;
        }
        System.out.println("starting the pickup");
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
