package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AutoBalanceSwerve extends CommandBase {


    private Double desiredHeading = 0.0;
    private boolean fieldrelative = true;
    private boolean openLoop = false;

    
    private final PIDController headingController = 
        new PIDController(VisionConstants.kTurnP, VisionConstants.kTurnI, VisionConstants.kTurnD);
       

    private double rotation;
    private Translation2d translation = new Translation2d(0 , 0);

    
    private Swerve s_Swerve;

    /**
     * Driver control
     */
    public AutoBalanceSwerve(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

     
    }

    @Override
    public void execute() {

 
        rotation = headingController.calculate(s_Swerve.getHeading().getDegrees(), desiredHeading);  
        if (s_Swerve.getPitch() > 9) {
            translation = new Translation2d(-0.3, 0.0); // Speed is in Meters/s
        } else if (s_Swerve.getPitch() < -9) {
            translation = new Translation2d(0.3, 0);
        } else {
            translation = new Translation2d(0 , 0);
        }


        s_Swerve.drive(translation, rotation, fieldrelative, openLoop);
    }
}
