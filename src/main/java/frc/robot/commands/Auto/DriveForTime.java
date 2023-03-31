// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.enums.AutoBalanceDirection;

public class DriveForTime extends CommandBase {
  double endTime;
  double timeout;
  DriveSubsystem m_DriveSubsystem;
  PIDController m_PIDController;
  boolean isLeveled;
  double curTime; 
  AutoBalanceDirection direction;
  double targetTime;
  boolean stoppedOnce = false;
  double levelAngle = -1.3;
  double speed;
  /** Creates a new ProtoypeChargLeveling. */
  public DriveForTime(DriveSubsystem DriveSubsystem, double seconds, double speed) {

    direction = AutoBalanceDirection.BACKWARD;
    m_DriveSubsystem = DriveSubsystem;
    targetTime = 0;
    timeout  = seconds;
    this.speed = speed;
    
  

    addRequirements(m_DriveSubsystem);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTimer();

    switch (direction){
      case FORWARD:
        m_DriveSubsystem.drive(-speed, 0, 0, true, true);
        m_DriveSubsystem.drive(-speed, 0, 0, true, true);
        break;
      case BACKWARD:
        m_DriveSubsystem.drive(speed, 0,0, true, true);
        m_DriveSubsystem.drive(speed, 0,0, true, true);
       break;
    }
    
    
      
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* 
    double angle  = m_DriveSubsystem.getPitch() - levelAngle;
    double stopDrivingAngle = 13.1;
    double levelThreshold = 1.5;

    
    
    switch(direction){

      case BACKWARD:
        if(angle <= -stopDrivingAngle && !stoppedOnce){
          m_DriveSubsystem.drive(-.05,0,0,true,true);
          stoppedOnce = true;
          curTime = System.currentTimeMillis();
        }

        else if(stoppedOnce && angle > - 4 && System.currentTimeMillis() - 2000 > curTime){
          m_DriveSubsystem.setX();
        }
        break;

      case FORWARD:
        if(angle >= stopDrivingAngle && !stoppedOnce){
          m_DriveSubsystem.drive(.05,0,0,true, true);
          curTime = System.currentTimeMillis();
          stoppedOnce = true;
          }
        if(stoppedOnce && angle < 4 && System.currentTimeMillis() - 2000 > curTime){
          m_DriveSubsystem.setX();
        }
        break;
        */



    }
    
    

  
  
    
  
    
  

  private void startTimer(){
    endTime = Timer.getFPGATimestamp() + timeout;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystem.drive(0,0,0,true,true);


    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return  Timer.getFPGATimestamp() >= endTime;
  
    }
  }

