// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;


public class AutoDriveFor extends CommandBase {
  double endTime;
  double timeout;
  double xSpeed;
  double ySpeed;
  double thetaSpeed;
  DriveSubsystem m_DriveSubsystem;
  boolean fieldRelative;
  /** Creates a new AutoDriveFor. */
  public AutoDriveFor(DriveSubsystem DriveSubsystem, double xSpeed, double ySpeed, double thetaSpeed, boolean fieldRelative, double seconds) {
    m_DriveSubsystem = DriveSubsystem;
    timeout = seconds;
    this.xSpeed = xSpeed/DriveConstants.kMaxSpeedMetersPerSecond;
    this.ySpeed = ySpeed/DriveConstants.kMaxSpeedMetersPerSecond;
    this.thetaSpeed = thetaSpeed;
    this.fieldRelative = fieldRelative;
    addRequirements(m_DriveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTimer();
    m_DriveSubsystem.drive(xSpeed, ySpeed, thetaSpeed, fieldRelative, true);
    m_DriveSubsystem.drive(xSpeed, ySpeed, thetaSpeed, fieldRelative, true);

  }

  private void startTimer(){
    endTime = Timer.getFPGATimestamp() + timeout;
  }

  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_DriveSubsystem.drive(0,0,0,false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() >= endTime;
  }
}
