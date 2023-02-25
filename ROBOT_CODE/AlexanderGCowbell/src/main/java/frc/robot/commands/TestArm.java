package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TestArm extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private ArmSubsystem s_armSubsystem;
    private XboxController controller;
    private int proximalAxis;
    private int distalAxis;


    /**
     * Driver control
     */
    public TestArm(ArmSubsystem s_armSubsystem, XboxController controller, int proximalAxis, int distalAxis) {
        this.s_armSubsystem = s_armSubsystem;
        addRequirements(s_armSubsystem);

        this.controller = controller;
        this.proximalAxis = proximalAxis;
        this.distalAxis = distalAxis;

    }

    @Override
    public void execute() {
        double pAxis = -controller.getRawAxis(proximalAxis) * Math.abs(controller.getRawAxis(proximalAxis));
        double dAxis = -controller.getRawAxis(distalAxis) * Math.abs(controller.getRawAxis(distalAxis));
  
        
        /* Deadbands 
        yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis;
         */

       // translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
        //rotation = rAxis * Constants.Swerve.maxAngularVelocity;
        s_armSubsystem.testArmMotors(pAxis, dAxis);
    }
}
