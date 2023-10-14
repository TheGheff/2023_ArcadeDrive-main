// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveDistanceCommand extends CommandBase {
  /** Creates a new DriveDistanceCommand. */
  private final DriveSubsystem m_drive;
  private final double m_distance;
  private final double m_speed;


  public DriveDistanceCommand(double inches, DriveSubsystem drive, double speed) {
    m_distance = inches;
    m_speed = speed;
    m_drive = drive;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.arcadeDrive(m_speed,0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.signum(m_distance) == 1)
    {
      return (Math.abs(m_drive.getAverageEncoderDistance()) >= m_distance);
    } 
    else
    {
      return (m_drive.getAverageEncoderDistance()) <= m_distance;
    }
  }
}
