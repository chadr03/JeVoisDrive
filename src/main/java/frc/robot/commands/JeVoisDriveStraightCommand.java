/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class JeVoisDriveStraightCommand extends Command {
  public JeVoisDriveStraightCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double move = -Robot.oi.driveStick.getY();
    double turn = Robot.oi.driveStick.getTwist();
    if (Robot.jSubsystem.isTargetVisable()){
      if(Robot.driveSubsystem.getDistance()>12){
        Robot.driveSubsystem.driveStraightJevois(0.7, 0.0025, 0.45);//(0.0, 0.003, 0.45)
        }else{
        Robot.driveSubsystem.driveStraightJevois(0.5, 0.0025, 0.45);
        }
    }else{
      Robot.driveSubsystem.teleopDrive(.7*move, turn);
    }
  }
{}
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
