package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.data.mp.*;

public class ArmPickupTestCommand extends CommandBase {
    private ArmSubsystem armSubsystem;

    public ArmPickupTestCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
    }

    @Override
	public void initialize() {
        ArmPath path = ShelfPickup.getArmPath();
        armSubsystem.startArmMovement(path); 
        armSubsystem.intake();
	}


    @Override
    public void end(boolean interrupted) {
        armSubsystem.holdIntake();
        armSubsystem.reverseArmMovment();
    }
}
