// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;
import frc.robot.Constants.ManipulatorConstants;

public class WristToDrive extends CommandBase {
  Wrist m_wrist;
  double targetPosition = ManipulatorConstants.kdriveEncoderPosition;
  /** Creates a new WristToDrive. */
  public WristToDrive(Wrist wrist) {
    m_wrist = wrist;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_wrist.goTo(targetPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_wrist.atPosition(targetPosition);
  }
}
