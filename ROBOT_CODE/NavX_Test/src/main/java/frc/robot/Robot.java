// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;

/**
 * This is a sample program to demonstrate how to use a gyro sensor to make a robot drive straight.
 * This program uses a joystick to drive forwards and backwards while the gyro is used for direction
 * keeping.
 */
public class Robot extends TimedRobot {
  public static final AHRS m_gyro = new AHRS(SPI.Port.kMXP, (byte) 200);

  public static long millis = System.currentTimeMillis();

  @Override
  public void robotInit() {
    m_gyro.reset();
    m_gyro.zeroYaw();
  }

  public static boolean doSD(){
    long now = System.currentTimeMillis();
    if(now - millis > 500){
      millis = now;
      return true;
    }
    return false;
  }

  @Override 
  public void teleopInit(){
    m_gyro.reset();
  }

  /**
   * The motor speed is set from the joystick while the DifferentialDrive turning value is assigned
   * from the error between the setpoint and the gyro angle.
   */
  @Override
  public void teleopPeriodic() {
    //double yaw = 360 - m_gyro.getYaw();
    double angle = m_gyro.getAngle();

    if(Robot.doSD()){
      double heading = Math.IEEEremainder(angle, 360) * - 1.0;
      SmartDashboard.putNumber("Gyro -> ANGLE ", heading);
      System.out.println("\n\nGyro -> HEADING " + heading + "\n\n\n");

    }
  }
}
