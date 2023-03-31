// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveDistance extends CommandBase {
  /** Creates a new AutoDriveDistance. */
  DriveSubsystem m_driveSubsystem;
  double distance;
  double speed;
  double distanceDriven;
  public AutoDriveDistance(DriveSubsystem driveSubsystem, double distance, double speedMPS) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;
    this.distance = distance;
    speed = speedMPS/DriveConstants.kMaxSpeedMetersPerSecond;
    distanceDriven = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveSubsystem.drive(speed, 0, 0, true, true);
    m_driveSubsystem.drive(speed, 0, 0, true, true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}
    
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_driveSubsystem.drive(0,0,0,true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_driveSubsystem.getDistanceDriven() >= distance;
  }
}
