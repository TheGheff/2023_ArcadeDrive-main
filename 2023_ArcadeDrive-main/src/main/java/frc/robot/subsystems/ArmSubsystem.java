// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

  private CANSparkMax m_Arm = new CANSparkMax(ArmConstants.kArmMotorPort, MotorType.kBrushless);
  //private CANSparkMax m_Grabber = new CANSparkMax(ArmConstants.kGrabberMotorPort, MotorType.kBrushless); 

  private SparkMaxPIDController m_pidControllerArm = m_Arm.getPIDController();  
  //private SparkMaxPIDController m_pidControllerGrabber = m_Grabber.getPIDController();  

    // The left-side Primary drive encoder
    private  RelativeEncoder m_ArmEncoderP = m_Arm.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    // The right-side Primary drive encoder
    //private  RelativeEncoder m_GrabberEncoderP = m_Grabber.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

  public ArmSubsystem() {
    //m_Grabber.setInverted(true);
   // m_Arm.setOpenLoopRampRate(2);
  }

  public double getPosition() {
    return m_ArmEncoderP.getPosition();
  }

  public void setHomPosition() {
    m_ArmEncoderP.setPosition(0);
  }

  public void setSpeed(double speed) {
    m_Arm.set(speed);
  }
/*
  public void grabberSetspeed(double speed) {
    m_Grabber.set(speed);
  }

  public void grabberClose(double speed) {
    m_Grabber.set(speed);
  }

  public void grabberOpen(double speed) {
    m_Grabber.set(speed);
  }

  public double getGrabberCurrent() {
    return m_Grabber.getOutputCurrent();
  }
*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
