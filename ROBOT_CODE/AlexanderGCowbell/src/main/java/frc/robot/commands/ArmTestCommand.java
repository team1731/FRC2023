package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.data.mp.*;

public class ArmTestCommand extends CommandBase {
    private ArmSubsystem armSubsystem;

    public ArmTestCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
    }

    @Override
	public void initialize() {
        int numberOfPoints = 0;
        double[][] proximalPoints = new double[][]{};
        double[][] distalPoints = new double[][]{};
        double[][] wristPoints = new double[][]{};
        ArmPath path = new ArmPath(numberOfPoints, proximalPoints, distalPoints, wristPoints);
        armSubsystem.startArmMovement(path); 
	}

    @Override
    public void end(boolean interrupted) {
        armSubsystem.reverseArmMovment();
    }
}
