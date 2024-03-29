// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kLeftMotor1Port = 3;
    public static final int kLeftMotor2Port = 4;
    public static final int kRightMotor1Port = 1;
    public static final int kRightMotor2Port = 2;

    public static final int[] kLeftEncoderPorts = new int[] {3, 4};
    public static final int[] kRightEncoderPorts = new int[] {1, 2};
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    public static final int kEncoderCPR = 42;
    public static final double kWheelDiameterInches = 6;
    public static final double kGeaeboxRatio = 12.75;//for toughbox mini
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        //(kWheelDiameterInches * Math.PI) / (double) kEncoderCPR; // Original math
        (kWheelDiameterInches * Math.PI) / ((double) kEncoderCPR*kGeaeboxRatio); 

    public static final boolean kGyroReversed = false;

    public static final double kStabilizationP = 1;
    public static final double kStabilizationI = 0.5;
    public static final double kStabilizationD = 0;

    public static final double kTurnP = 1;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;

    public static final double kMaxTurnRateDegPerS = 100;
    public static final double kMaxTurnAccelerationDegPerSSquared = 300;

    public static final double kTurnToleranceDeg = 5;
    public static final double kTurnRateToleranceDegPerS = 10; // degrees per second
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }


  public static final class ArmConstants {
    public static final int kArmMotorPort = 5;
    public static final int kGrabberMotorPort = 6;

    public static final int kEncoderCPR = 42;
    public static final double kWheelDiameterInches = 7;
    public static final double kGearBoxRatio = 100;
    public static final double kChainRatio = 5;//(12/60)
    public static final double kEncoderDistancePerPulse = 
      (kWheelDiameterInches * Math.PI) / ((double) kChainRatio*kGearBoxRatio);

    public static final double kMoveP = 1;
    public static final double kMoveI = 0;
    public static final double kMoveD = 0;  

    public static final double kTurnToleranceDeg = 10;
    public static final double kTurnRateToleranceDegPerS = 10; // degrees per second
  }


}
