package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {

    private final PIDController headingController = 
        new PIDController(VisionConstants.kTurnP, VisionConstants.kTurnI, VisionConstants.kTurnD);

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private Swerve s_Swerve;
    private XboxController controller;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;

    /**
     * Driver control
     */
    public TeleopSwerve(Swerve s_Swerve, XboxController controller, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
        headingController.enableContinuousInput(-180.0, 180.0);
    }

    @Override
    public void execute() {
        double yAxis = -controller.getRawAxis(translationAxis) ;
        double xAxis = -controller.getRawAxis(strafeAxis) ;
        double rAxis = -controller.getRawAxis(rotationAxis) ;
        SmartDashboard.putNumber("yaxis before ",yAxis);
        SmartDashboard.putNumber("xaxis before ",xAxis);
        SmartDashboard.putNumber("raxis before ",rAxis);

        /* Deadbands */
        yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : (yAxis > 0? (yAxis - Constants.stickDeadband ): (yAxis + Constants.stickDeadband));
        yAxis *= Math.abs(yAxis);
        yAxis = yAxis/(1-Constants.stickDeadband);
        xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : (xAxis > 0? (xAxis - Constants.stickDeadband ): (xAxis + Constants.stickDeadband));
        xAxis *= Math.abs(xAxis);
        xAxis = xAxis/(1-Constants.stickDeadband);
        rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : (rAxis > 0? (rAxis - Constants.stickDeadband ): (rAxis + Constants.stickDeadband));
        rAxis *= Math.abs(rAxis);
        rAxis = rAxis/(1-Constants.stickDeadband);

        SmartDashboard.putNumber("yaxis after ",yAxis);
        SmartDashboard.putNumber("xaxis after ",xAxis);
        SmartDashboard.putNumber("raxis after ",rAxis);
        translation = new Translation2d(yAxis , xAxis).times(Constants.Swerve.maxSpeed);


        // If the right stick is neutral - this code should lock onto the last known
        // heading
        if (Math.abs(rAxis) == 0.0) {
            if (s_Swerve.lockedHeading == null) {
            //    System.out.println("Resetting the heading");
                headingController.reset();
                s_Swerve.lockedHeading = s_Swerve.getHeading().getDegrees();
            }

            rotation = headingController.calculate(s_Swerve.getHeading().getDegrees(), s_Swerve.lockedHeading);
            //    System.out.println("rotation from controller"+ rotation);
            rotation  = (Math.abs(rotation) < .1) ? 0 : rotation;
        } else {

            s_Swerve.lockedHeading = null;
            rotation = rAxis * Constants.Swerve.maxAngularVelocity;
            //   System.out.println("rotation from stick" + rotation);
        }

        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
    }
}
