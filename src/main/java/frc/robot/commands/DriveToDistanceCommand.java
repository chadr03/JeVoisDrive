/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class DriveToDistanceCommand extends Command {
  private final PIDController m_pid;

  public DriveToDistanceCommand(double distance, double kP, double kI, double kD) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveSubsystem);
  
    m_pid = new PIDController(kP, kI, kD, new PIDSource() {
      PIDSourceType m_sourceType = PIDSourceType.kDisplacement;

      @Override
      public double pidGet() {
        return Robot.driveSubsystem.getDistance();
      }

      @Override
      public void setPIDSourceType(PIDSourceType pidSource) {
        m_sourceType = pidSource;
      }

      @Override
      public PIDSourceType getPIDSourceType() {
        return m_sourceType;
      }
    //}, d -> Robot.driverain.drive(0, d));
   }, d -> Robot.driveSubsystem.teleopDrive(-d, 0.0));

  
  m_pid.setAbsoluteTolerance(0.5);
  m_pid.setToleranceBuffer(1);
  m_pid.setSetpoint(distance);
  m_pid.setOutputRange(-0.7, 0.7);
  
  
  
  
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
     m_pid.reset();
     m_pid.enable();
     System.out.println("PID enabled.\n");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putNumber("Distance Setpoint", m_pid.getSetpoint());
  	SmartDashboard.putNumber("Distance Error", m_pid.getError());
  	SmartDashboard.putBoolean("Distance PID is On Target", m_pid.onTarget());
  	SmartDashboard.putBoolean("Distance PID is Executing", m_pid.isEnabled());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (m_pid.onTarget() || (Math.abs(Robot.oi.driveStick.getY())>0.2));
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    m_pid.disable();
    Robot.driveSubsystem.teleopDrive(0,0);
 
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    m_pid.disable();
  }
}
