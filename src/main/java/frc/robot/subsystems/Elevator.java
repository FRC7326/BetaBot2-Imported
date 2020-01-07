/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.mainelvcommand;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  // public PWMTalonSRX elevator_motor = new PWMTalonSRX(RobotMap.TalonSRX);
  //private PWMTalonSRX chainmotor = new TalonSRX(RobotMap.ChainmotorPort);
  // Put methods for controlling this elevator_motorsubsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new mainelvcommand());

  }
  public class ElevatorModule{
    private TalonSRX mLift;
    public ElevatorModule(int kDriveID) {
      mLift = new TalonSRX(kDriveID);
    }
  }


  
//did javi put this?
public void setSpeed(double speed) {
    //chainmotor.setSpeed(speed);
    //originally 0.5 was the word "speed"
}
}