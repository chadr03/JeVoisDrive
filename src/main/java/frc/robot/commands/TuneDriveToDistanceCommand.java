/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TuneDriveToDistanceCommand extends Command {
  public TuneDriveToDistanceCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    double distance = SmartDashboard.getNumber("Distance Setpoint", 20.0);
    double kP = SmartDashboard.getNumber("Distance P", 0);
    double kI = SmartDashboard.getNumber("Distance I", 0);
    double kD = SmartDashboard.getNumber("Distance D", 0);	  
    DriveToDistanceCommand driveCmd=new DriveToDistanceCommand(distance, kP, kI, kD);
    driveCmd.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
