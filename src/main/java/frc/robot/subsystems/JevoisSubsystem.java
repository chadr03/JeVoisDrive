/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.Subsystem;


/**
 * Add your docs here.
 */
public class JevoisSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private int targetVisible=0;
  private double tgtX = 0.0;
  private double tgtY = 0.0;

  private SerialPort jevois;

  public JevoisSubsystem(){
    try{
      jevois = new SerialPort(115200, SerialPort.Port.kUSB);
      System.out.println("Connected on kUSB!");
    } catch (Exception e) {
      System.out.println("Failed to connect on kUSB, trying kUSB1");

      try {
        jevois = new SerialPort(115200, SerialPort.Port.kUSB1);
        System.out.println("Connected on kUSB1");
      } catch (Exception e1) {
        System.out.println("Failed to connect on kUSB1, trying kUSB2");

        try {
          jevois = new SerialPort(115200, SerialPort.Port.kUSB2);
          System.out.println("Connected on kUSB2"); 
        } catch (Exception e2) {
          System.out.println("Failed to connect on kUSB2, all connection attemps Failed");
        }
      }
    }
  }


  public String getJevoisPacket(){
    if(jevois.getBytesReceived() > 0){
      return jevois.readString();
    }else{
      return "";
    }
  }

  public void getJevoisData(){
    if(jevois.getBytesReceived() > 0){
      String[] jevoisData=jevois.readString().split(",");
      targetVisible = Integer.parseInt(jevoisData[0]);
      tgtX= Double.parseDouble(jevoisData[1]);
      tgtY= Double.parseDouble(jevoisData[2]);
   
    }
  }

  public boolean isTargetVisable(){
    getJevoisData();
    if (targetVisible==1){
      return true;
    } else{
      return false;
    }
  }

  public double getTgtX(){
    getJevoisData();
    return tgtX;
  }

  public double getTgtY(){
    getJevoisData();
    return tgtY;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
