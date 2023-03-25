// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants.ArmConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveArm extends PIDCommand {
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
  public MoveArm(double position, ArmSubsystem arm) {
    super(
        new PIDController(ArmConstants.kMoveP, ArmConstants.kMoveI, ArmConstants.kMoveD),
        // Close loop on heading
        arm::getPosition,
        // Set reference to target
        position,
        // Pipe output to turn robot
        output -> arm.setSpeed(output),
        // Require the drive
        arm);

    // Set the controller to be continuous (because it is an rotation count controller)
    getController().enableContinuousInput(-5250, 5250); // # tick, 42 per  rotations 125 rotations in each direction
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(ArmConstants.kTurnToleranceDeg, ArmConstants.kTurnRateToleranceDegPerS);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }
}