/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;


public class DriveManuallyCommand extends Command {
   public DriveManuallyCommand() {
    //plug in the desired numbers into time, lSpeed, and rSpeed
  

    // Use requires() here to declare subsystem dependencies
     requires(Robot.driveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double move = -1*Robot.m_oi.nescontroller.getY();
    double turn = Robot.m_oi.nescontroller.getX();
    Robot.driveSubsystem.manualDrive(move, turn);


    if(Robot.m_oi.getUp()){
      Robot.driveSubsystem.MoveElevator(0.3);
    }

    if(Robot.m_oi.getDown()){
      Robot.driveSubsystem.MoveElevator(-0.3);
    }
  }
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
    end();
  }
}
