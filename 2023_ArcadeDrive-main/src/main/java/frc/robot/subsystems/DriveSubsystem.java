// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  private CANSparkMax m_LeftPrimary = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
  private CANSparkMax m_LeftSecondary = new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless); 

  private final MotorControllerGroup m_leftMotors =
      new MotorControllerGroup(
        m_LeftPrimary,
        m_LeftSecondary);

  // The motors on the right side of the drive.
  private CANSparkMax m_RightPrimary = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);
  private CANSparkMax m_RightSecondary = new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless); 
  
  
  //m_RightPrimary = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushed);
  //m_RightSecondary = new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushed);
  private final MotorControllerGroup m_rightMotors =
      new MotorControllerGroup(
        m_RightPrimary,
        m_RightSecondary);


  private SparkMaxPIDController m_pidControllerLeft = m_LeftPrimary.getPIDController();  
  private SparkMaxPIDController m_pidControllerRight = m_RightPrimary.getPIDController();  



  


  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The left-side Primary drive encoder
  private  RelativeEncoder m_leftEncoderP = m_LeftPrimary.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
  // The right-side Primary drive encoder
  private  RelativeEncoder m_rightEncoderP = m_RightPrimary.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
 // The left-side Secondary drive encoder
 private  RelativeEncoder m_leftEncoderS = m_LeftSecondary.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
 // The right-side Secondary drive encoder
 private  RelativeEncoder m_rightEncoderS = m_RightSecondary.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);


  // The gyro sensor
  AHRS ahrs = new AHRS(SPI.Port.kMXP);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    //m_rightMotors.setInverted(true);

    // Sets the distance per pulse for the encoders

    //m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    //m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftEncoderP.setPosition(0);
    m_rightEncoderP.setPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return ((m_leftEncoderP.getPosition()*DriveConstants.kGeaeboxRatio) + (m_rightEncoderP.getPosition()*DriveConstants.kGeaeboxRatio)) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public RelativeEncoder getLeftEncoder() {
    return m_leftEncoderP;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public RelativeEncoder getRightEncoder() {
    return m_rightEncoderP;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    ahrs.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(ahrs.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return ahrs.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
