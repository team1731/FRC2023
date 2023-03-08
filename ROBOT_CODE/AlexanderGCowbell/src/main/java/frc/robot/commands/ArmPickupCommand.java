package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.HighPickup;
import frc.robot.Constants.ArmStateConstants;
import frc.robot.state.arm.ArmSequence;
import frc.robot.state.arm.ArmStateMachine;
import frc.robot.state.arm.ArmStateMachine.MovementType;
import frc.data.mp.*;

public class ArmPickupCommand extends CommandBase {
    private ArmStateMachine stateMachine;
    private ArmSequence sequence;
    private Joystick joystick;
    private int distalAxis;
    private boolean adjustWrist = false;

    public ArmPickupCommand(ArmStateMachine stateMachine, ArmSequence sequence, Joystick joystick, int distalAxis) {
        this.stateMachine = stateMachine;
        this.sequence = sequence;
        this.joystick = joystick;
        this.distalAxis = distalAxis;
    }

    @Override
	public void initialize() {
        ArmPath path = null;
        if(sequence == ArmSequence.PICKUP_HIGH && stateMachine.getGamePiece() == GamePiece.CONE && stateMachine.getHighPickup() == HighPickup.FEEDER) {
            path = PickupHighConeFeeder.getArmPath();
        } else if(sequence == ArmSequence.PICKUP_HIGH && stateMachine.getGamePiece() == GamePiece.CONE && stateMachine.getHighPickup() == HighPickup.SHELF) {
            path = PickupHighCone.getArmPath();
        } if(sequence == ArmSequence.PICKUP_HIGH && stateMachine.getGamePiece() == GamePiece.CUBE) {
            path = PickupHighCube.getArmPath();
        } else if(sequence == ArmSequence.PICKUP_LOW && stateMachine.getGamePiece() == GamePiece.CONE) {
            path = PickupLowCone.getArmPath();
            adjustWrist = true;
        } else if(sequence == ArmSequence.PICKUP_LOW && stateMachine.getGamePiece() == GamePiece.CUBE) {
            path = PickupLowCube.getArmPath();
            adjustWrist = true;
        } else if (sequence == ArmSequence.FLIP_CONE) {
            stateMachine.pickup(ArmStateConstants.coneFlipFlexPosition);
            adjustWrist = true;
        } else if (sequence == ArmSequence.PICKUP_DOWNED_CONE) {
            path = PickupFloorCone.getArmPath();
            adjustWrist = true;
        }
        
        // Note: distal joystick doubles for distal arm and wrist adjustment dependending on the path being run
        stateMachine.addJoystickControl(joystick, distalAxis, adjustWrist);

        if(path != null && sequence == ArmSequence.PICKUP_DOWNED_CONE) {
            stateMachine.pickup(path, MovementType.PICKUP_DOWNED_CONE);
        } else if(path != null) {
            stateMachine.pickup(path);
        }
	}

    @Override
    public void end(boolean interrupted) {
        stateMachine.buttonReleased();
    }
}
