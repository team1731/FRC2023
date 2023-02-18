package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GamePiece;
import frc.robot.state.arm.ArmSequence;
import frc.robot.state.arm.ArmStateMachine;
import frc.robot.state.arm.ArmStateMachine.Status;
import frc.robot.state.arm.ArmStateMachine.MovementType;
import frc.data.mp.*;

public class ArmPickupCommand extends CommandBase {
    private ArmStateMachine stateMachine;
    private ArmSequence sequence;

    public ArmPickupCommand(ArmStateMachine stateMachine, ArmSequence sequence) {
        this.stateMachine = stateMachine;
        this.sequence = sequence;
    }

    @Override
	public void initialize() {
        if(stateMachine.getStatus() == Status.RUNNING && stateMachine.getMovementType() != MovementType.PICKUP) {
            System.out.println("WARNING: cannot command a pickup when arm state is already running a different movement");
            return;
        } else if(stateMachine.getStatus() == Status.RUNNING) {
            stateMachine.restartMovement();
        }

        // no movement is currently running, load the correct path and start the pickup
        ArmPath path = null;
        if(sequence == ArmSequence.PICKUP_HIGH) {
            path = PickupHigh.getArmPath();
        } else if(stateMachine.getGamePiece() == GamePiece.CONE) {
            path = PickupLowCone.getArmPath();
        } else {
            path = PickupLowCube.getArmPath();
        }

        stateMachine.pickup(path);
	}

    @Override
    public void end(boolean interrupted) {
        stateMachine.buttonReleased();
    }
}
