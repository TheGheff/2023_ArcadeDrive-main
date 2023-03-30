// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import static edu.wpi.first.wpilibj.PS4Controller.Button;
import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

import frc.robot.commands.TurnToAngle;
import frc.robot.commands.TurnToAngleProfiled;
import frc.robot.commands.MoveArm;
import frc.robot.commands.CloseGrabber;
import frc.robot.commands.OpenGrabber;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final   ArmSubsystem m_Arm = new ArmSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () ->
                m_robotDrive.arcadeDrive(
                    -m_driverController.getLeftY(), -m_driverController.getRightX()), 
            m_robotDrive));



    m_Arm.setDefaultCommand(
        new RunCommand(
            () -> 
                m_Arm.setSpeed(m_operatorController.getLeftY()),
                m_Arm));


  };



  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link PS4Controller}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_robotDrive.setMaxOutput(0.125); //totally inthe wrong place #fix

    // Drive at half speed when the right bumper is held
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.125)))//.25
        .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.25)));  //1

//Does not work        
    // Stabilize robot to drive straight with gyro when left bumper is held
/*    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .whileTrue(
            new PIDCommand(
                new PIDController(
                    DriveConstants.kStabilizationP,
                    DriveConstants.kStabilizationI,
                    DriveConstants.kStabilizationD),
                // Close the loop on the turn rate
                m_robotDrive::getTurnRate,
                // Setpoint is 0
                0,
                // Pipe the output to the turning controls
                output -> m_robotDrive.arcadeDrive(.3, output),
                // Require the robot drive
                m_robotDrive));
*/
/*
    // Turn to 90 degrees when the 'X' button is pressed, with a 5 second timeout
    new JoystickButton(m_driverController, Button.kA.value)
        .onTrue(new TurnToAngle(90, m_robotDrive).withTimeout(1));

    // Turn to -90 degrees with a profile when the Circle button is pressed, with a 5 second timeout
    new JoystickButton(m_driverController, Button.kB.value)
        .onTrue(new TurnToAngleProfiled(-90, m_robotDrive).withTimeout(1));
*/

 // Move arm up
    new JoystickButton(m_operatorController, Button.kB.value)
    .onTrue( new InstantCommand(() ->m_Arm.setSpeed(.5)))
    .onFalse( new InstantCommand(() ->m_Arm.setSpeed(0)));
//move arm down
    new JoystickButton(m_operatorController, Button.kA.value) 
    .onTrue( new InstantCommand(() ->m_Arm.setSpeed(-.5)))
    .onFalse( new InstantCommand(() ->m_Arm.setSpeed(0)));


 // Grabber Open
    new JoystickButton(m_operatorController, Button.kX.value)
    .onTrue( new CloseGrabber(m_Arm))//new InstantCommand(() ->m_Arm.grabberClose(-.5)))
    .onFalse( new InstantCommand(() ->m_Arm.grabberClose(0)));


 // Grabber Close
    new JoystickButton(m_operatorController, Button.kY.value) 
    .onTrue( new OpenGrabber(m_Arm))//new InstantCommand(() ->m_Arm.grabberOpen(.5)))
    .onFalse( new InstantCommand(() ->m_Arm.grabberOpen(0)));
        
        

    //this is a BAD hack #fix
//    new JoystickButton(m_operatorController, Button.kA.value) 
 //  .onTrue (new RunCommand(() ->m_Arm.setSpeed(m_operatorController.getLeftY()),
 //               m_Arm) );


//*********Closed Loop*********UNTESTED
         

/*
    // Stabilize robot to drive straight with gyro when left bumper is held
    new JoystickButton(m_driverController, Button.kLeftBumper.value)
    .whileTrue(
        new PIDCommand(
            new PIDController(
                DriveConstants.kStabilizationP,
                DriveConstants.kStabilizationI,
                DriveConstants.kStabilizationD),
            // Close the loop on the turn rate
            m_robotDrive::getTurnRate,
            // Setpoint is 0
            0,
            // Pipe the output to the turning controls
            output -> m_robotDrive.arcadeDrive(.5, output),
            // Require the robot drive
            m_robotDrive));


/*  //Does not work
    // Turn to 90 degrees when the 'X' button is pressed, with a 5 second timeout
    new JoystickButton(m_driverController, Button.kA.value)
        .onTrue(new TurnToAngle(90, m_robotDrive).withTimeout(5));

    // Turn to -90 degrees with a profile when the Circle button is pressed, with a 5 second timeout
    new JoystickButton(m_driverController, Button.kB.value)
        .onTrue(new TurnToAngle(-90, m_robotDrive).withTimeout(5));


  


        
    new JoystickButton(m_operatorController, Button.kRightBumper.value)
        .onTrue( new MoveArm(-150,m_Arm));//.withTimeout(1));

     // Move arm up
     new JoystickButton(m_operatorController, Button.kLeftBumper.value)
     .onTrue( new MoveArm(-200,m_Arm));//.withTimeout(1));
*/
     

    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // no auto

    return new InstantCommand();
  }
}
