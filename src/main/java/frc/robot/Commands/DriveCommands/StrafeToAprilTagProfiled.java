// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.DriveCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Commands.PIDBase.ProfiledDoublePIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Cameras.AprilTagSubsystem;
import frc.robot.subsystems.Swerve.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** A command that will turn the robot to the specified angle using a motion profile. */
public class StrafeToAprilTagProfiled extends ProfiledDoublePIDCommand {
  
  // turn PID
  private static ProfiledPIDController m_PID1 = new ProfiledPIDController(
    DriveConstants.kYawP, DriveConstants.kYawI, DriveConstants.kYawD,
    new TrapezoidProfile.Constraints(
                DriveConstants.kMaxYawRateDegPerS,
                DriveConstants.kMaxYawAccelerationDegPerSSquared));

  // strafe PID
  private static ProfiledPIDController m_PID2 = new ProfiledPIDController(
    DriveConstants.kYawP/2, DriveConstants.kYawI, DriveConstants.kYawD,
    new TrapezoidProfile.Constraints(
                DriveConstants.kMaxYawRateDegPerS,
                DriveConstants.kMaxYawAccelerationDegPerSSquared));


  private AprilTagSubsystem m_aprilTags;

  private static boolean m_shuffleboardLoaded = false;
  /**
   * Turns to robot to the specified angle using a motion profile.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
  public StrafeToAprilTagProfiled(AprilTagSubsystem aprilTags, DriveSubsystem drive) {
    super(
        m_PID1, m_PID2,
        // rotate and strafe
        aprilTags::getBestAprilTagYaw, aprilTags::getBestAprilTag3dAngle,
        // Set reference to target
        0, 180,
        // Pipe output to turn robot
        (output, setpoint) -> drive.drive(0, -output.x2, output.x1, false, true),
        // Require the drive
        drive);

    // Set the controller to be continuous (because it is an angle controller)
    getController1().enableContinuousInput(-180, 180);
    getController2().enableContinuousInput(-180, 180);

    getController1().setTolerance(DriveConstants.kYawToleranceDeg, DriveConstants.kYawRateToleranceDegPerS);
    getController2().setTolerance(DriveConstants.kYawToleranceDeg, DriveConstants.kYawRateToleranceDegPerS);
      
    // Add the PID to dashboard
    if (!m_shuffleboardLoaded) {
      ShuffleboardTab turnTab = Shuffleboard.getTab("Drivebase");
      turnTab.add("Double PID 1", m_PID1);
      turnTab.add("Double PID 2", m_PID2);
      m_shuffleboardLoaded = true;  // so we do this only once no matter how many instances are created
    }
    m_aprilTags = aprilTags;
  
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    if (getController2().atGoal()) {System.out.println("done strafing to April Tag");}
    return getController1().atGoal() && getController2().atGoal();
  }

  @Override
  public void execute() {
    System.out.println("stafing to April Tag " + m_aprilTags.getBestAprilTag3dAngle());
    super.execute();
  }  
}
