package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.data.mp.*;

public class ArmScoreLowCommand extends CommandBase {
    private ArmSubsystem armSubsystem;

    public ArmScoreLowCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
    }

    @Override
	public void initialize() {
        System.out.println("Score low command fired!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        ArmPath path = ScoreLow.getArmPath();
        armSubsystem.startArmMovement(path);
	}

    @Override
    public void end(boolean interrupted) {
        armSubsystem.eject();
        armSubsystem.reverseArmMovment();
    }
}
