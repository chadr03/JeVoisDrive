/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.TeleopDriveCommand;

/**
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  double xTargetSetpoint = 160.0;



  Spark left1 = new Spark(RobotMap.left1Port);
  Spark left2 = new Spark(RobotMap.left2Port);
  Spark right1 = new Spark(RobotMap.right1Port);
  Spark right2 = new Spark(RobotMap.right2Port);
  PowerDistributionPanel pdp = new PowerDistributionPanel();

  SpeedControllerGroup leftMaster = new SpeedControllerGroup(left1, left2);
  SpeedControllerGroup rightMaster = new SpeedControllerGroup(right1, right2);


  DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);


  Ultrasonic ut = new Ultrasonic(RobotMap.ultrasonicPingPort, RobotMap.ultrasonicEchoPort);

  public DriveSubsystem() {
    ut.setAutomaticMode(true);
  }

  public void teleopDrive(double move, double turn) {
    drive.arcadeDrive(move, turn);
  }

  public double getDistance(){
    SmartDashboard.putNumber("Test", ut.getRangeMM());
    return ut.getRangeInches();
  }

  public void driveStraightJevois(double power, double kP, double kF) {
    double targetError = (xTargetSetpoint - Robot.jSubsystem.getTgtX());
    kF = Math.copySign(kF,targetError);
    double turningValue = targetError * -kP - kF;
    
    
    // Invert the direction of the turn if we are going backwards
    //turningValue = Math.copySign(turningValue, power);
    //if (power<0) {
    //	turningValue=-1*turningValue;
    //}
    SmartDashboard.putNumber("TurningValue", turningValue);
    teleopDrive(power, turningValue);
}



  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new TeleopDriveCommand());


  }
}
