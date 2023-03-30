// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OpenGrabber extends CommandBase {
  private ArmSubsystem arm;
  public OpenGrabber( ArmSubsystem armSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    arm = armSub;
    armSub.grabberSetspeed(.5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.

    
    return (arm.getGrabberCurrent() > 1);
  }

  @Override
  public void end(boolean interrupted) {
    // End when the controller is at the reference.
    arm.grabberSetspeed(0);
    
  
  }
  
}
