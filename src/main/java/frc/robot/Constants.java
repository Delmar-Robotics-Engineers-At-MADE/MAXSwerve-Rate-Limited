// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(18.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 2;
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kFrontRightDrivingCanId = 1;
    public static final int kRearRightDrivingCanId = 4;

    public static final int kFrontLeftTurningCanId = 12;
    public static final int kRearLeftTurningCanId = 13;
    public static final int kFrontRightTurningCanId = 11;
    public static final int kRearRightTurningCanId = 14;

    public static final boolean kGyroReversed = false;

    // speeds
    public static final double kNormalSpeed = 0.8;
    public static final double kNormalYaw = 0.7;
    public static final double kTurboSpeed = 1.0;
    public static final Double kTurboYaw = 0.8;
    public static final double kCrawlSpeed = 0.3;
    public static final double kODriveSpeed = 0.5;
    public static final double kOSlowSpeed = 0.25;
    
    // TurnToAprilTagProfiled
    public static final double kYawP = 0.011;
    public static final double kYawI = 0.0;
    public static final double kYawD = 0.012;
    public static final double kMaxYawRateDegPerS = 8;
    public static final double kMaxYawAccelerationDegPerSSquared = 20;
    public static final double kYawToleranceDeg = 5;
    public static final double kYawRateToleranceDegPerS = 10;

    public static final double kStrafeP = 0.01;
    public static final double kStrafeI = 0.0;
    public static final double kStrafeD = 0.0;
    public static final double kMaxStrafeRateDegPerS = 1;
    public static final double kMaxStrafeAccelerationDegPerSSquared = 20;
    public static final double kStrafeToleranceDeg = 20;
    public static final double kStrafeRateToleranceDegPerS = 10;

    // DriveToAprilTag
    public static final double kDriveP = 0.1;
    public static final double kDriveI = 0.02;
    public static final double kDriveD = 0;
    public static final double kDriveToleranceDist = 0.15;

  }

  public static final class CameraConstants {
    // Turn to Gamepiece
    public static final double kGamepieceCenterPos = -10.5;
    public static final double CAMERA_HEIGHT_METERS = 0.9;
    public static final double TARGET_HEIGHT_METERS = 0.31;
    public static final double CAMERA_PITCH_RADIANS = -0.436;
    public static final double kSummerAprilTagDistance = 2.5; // meters
    public static final double kSummerAprilTagDistanceBackup = 1.5; // meters
    public static final double kSummerSearchForAprilTagYaw = -10; // degrees
    
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 12;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.1;
    public static int kOpperatorControllerPort = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class BalanceConstants {

    public static final double BEAM_BALANCED_GOAL_DEGREES = 0.0;
    public static final double BEAM_BALANACED_DRIVE_KP = 0.007;
    public static final double BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER = 1.2;
    public static final double BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES = 2.5;

  }

  public static final class EverybotConstants {
        /**
     * How many amps the intake can use while picking up
     */
    public static final int INTAKE_CURRENT_LIMIT_A = 25;

    /**
     * How many amps the intake can use while holding
     */
    public static final int INTAKE_HOLD_CURRENT_LIMIT_A = 5;

    /**
     * Percent output for intaking
     */
    public static final double INTAKE_OUTPUT_POWER = 0.3;

    /**
     * Percent output for holding
     */
    public static final double INTAKE_HOLD_POWER = 0.07;

    public static final  int INTAKE_ID = 5;

  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class SWORD_CONSTANTS {
    public static final int CLAW_ID = 6;
    public static int kTimeoutMs = 30;
    public static int kPIDLoopIdx = 0;
    public static int kPIDPositionIdx = 1;
    public static int kPIDPositionTolerance = 40;
    public final static Gains kGains_Velocity = new Gains( 0.05, 0.0, 0.0, 0.1,  300,  1.00);
    public final static Gains kGains_Position = new Gains( 5, 0.0, 0.7, 0.0,  300,  1.00);

    public static final double kYawP = 0.012;
    public static final double kYawI = 0.001;
    public static final double kYawD = 0.003;
    public static final double kMaxYawRateDegPerS = 8;
    public static final double kMaxYawAccelerationDegPerSSquared = 20;
    public static final double kYawToleranceDeg = 1;
    public static final double kYawRateToleranceDegPerS = 10;

    public static int kWaxMove = 3000;
    public static int kParryLeft = 637;
    public static int kParryRight = -637;
  }

  public static final class CLAW_CONSTANTS {
    public static final int CLAW_ID = 6;
    public static final long kInVelocity = -6000;
    public static final long kConeOutVelocity = 2048;
    public static final long kCubeOutVelocity = 10000;
    public static final long kShootVelocity = 4096;
    public static final long kStopVelocity = 0;
    public static final long kHoldVelocity = 300;
    public static final double kStallCurrent = -29;
    public static final long kStallVelocity = 20;
    public static final int kMotorStartupTime = 1;

    public static int kTimeoutMs = 30;
    public static int kPIDLoopIdx = 0;
    public static int kPIDPositionIdx = 1;
    public static int kPIDPositionTolerance = 10;
    //                                                    kP     kI    kD  kF             Iz     PeakOut 
    //public final static Gains kGains_Velocity = new Gains( 0.25, 0.0, 0.0, 0.08,  300,  1.00);
    // versa with encoder next to output shaft: 
    public final static Gains kGains_Velocity = new Gains( 0.05, 0.0, 0.0, 0.1,  300,  1.00);
    public final static Gains kGains_Position = new Gains( 3, 0.0, 0.0, 0.0,  300,  1.00);
  }

  public static final class UpperArmConstants {

    public static final int UPPER_ARM_MOTOR_ID = 0;

    public static final int kRevEncoderLimitLow = -20;
    public static final int kRevEncoderLimitHigh = -200;
    public static final double kRevEncoderP = 0.015;  // was 0.003 b4 added versa stage
    public static final double kRevEncoderI = 0;
    public static final double kRevEncoderD = 0.006;  // was 0.002 b4 added versa stage
    public static final double kRevEncoderTolerance = 20; 
    public static final double kRevEncoderMaxCountsPerS = 250;
    public static final double kRevEncoderMaxCountsPerSSquared = 250;
    public static final double kRevEncoderMaxCountsPerSTolerance = 100;

    public static final double kPotmeterP = 60; // was 20 b4 added versa stage
    public static final double kPotmeterI = 6;  // was 2 b4 added versa stage
    public static final double kPotmeterD = 0;
    public static final double kPotmeterTolerance = 0.001;  

    public static final double kHomePotmeterValue = 0.457;
    public static final double kHomeEncoderValue = 0.0;

    public static final double kMaxFalconPower = 0.60;  // was 0.20 b4 added versa stage

    public static final double kHighPosition = 769.50;
    public static final double kMidPosition = 233.75;
    public static final double kSummerReturnPosition = 620;
    public static final double kSummerIntakePosition = 0;

    // for testing only
    public static final double kFalconClosedLoopTolerance = 100;
    public static final double kFalconTestNudgeAmount = 6000;  // was 2000 b4 added versa stage
    public static final double kFalconP = 0.1; 

  }

  public static final class LowerArmConstants {

    public static final int LOWER_ARM_MOTOR_ID = 7;
    // public static final int LOWER_ARM_CHAIN_MOTOR_ID = 8;
    public static double kP = 5.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kFF = 0.0;
    public static double kMinOutput = -0.3;
    public static double kMaxOutput = 0.3;
    public static double kNudgeCounts = 0.07;
    public static double kTolerance = 0.005;
    public static double kSlowZone = 0.03;

    public static final double kHomePosition = 0.022;
    public static final double kFullExtension = 0.29;
    public static final double kFloorPosition = 0.28;
    public static final double kHighPosition = 0.5;
    public static final double kShootPosition = 20;
    public static final double kMidPosition = 0.106;
    public static final double kSSsPosition = 0.1;
    public static final double kManualSpeed = 70;
    public static final double minVelocity = 0;
    public static final double maxAccel = 1500;
    public static final double maxVelocity = 5000;
    public static final double allowedErr = 0;
  }

  public static final class ControllerConstants {

    public static final class DriverConstants {
      public static final int X_MODE = 3;
      public static final int TURBO = 2;
      //public static final int ZERO_HEADING = 6;
      public static final int CRAWL = 1;
      public static final int kHomeArms = 5;
      public static final int kAutoCubeMid = 4;
      public static final int kAutoCubeHigh = 6;
      // public static final int kDriveToTag = 8;
      // public static final int kAutoBalance = 12;
      // public static final int kSingleSubstation = 4;
      // public static final int kMidPosition = 5;
      // public static final int kLowerArmUp = 10;
      // public static final int kLowerArmDown = 9;

    }
    public static final class OpperatorConstants {
      public static final int PRIORITY_LEFT = 9;
      //public static final int kUpperArmManual = 12;
      public static final int kFloorMode = 3;
      public static final int kReverseIntake = 5;
      public static final int kHigh = 6;
      public static final int kMid = 4;
      //public static final int kClawTest = 1;
      public static final int kHomeArms = 10;
      //public static final int ksetUpperArm = 7;
      public static final int kNudgeUp = 12;
      public static final int kNudgeDown = 11;
      //public static final int kMoveUpperArmToTarget = 5;
      //public static final int kShootPosition = 1;
      //public static final int kHighPosition = 6;
      //public static final int kMidPosition = 8;
      //public static final int kClawIn = 5;
      public static final int kCube = 1;
      public static final int kCone = 2;
    }
  }
}
