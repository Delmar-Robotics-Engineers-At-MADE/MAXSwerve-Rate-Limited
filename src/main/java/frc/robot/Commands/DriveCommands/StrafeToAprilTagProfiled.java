// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.DriveCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Cameras.AprilTagSubsystem;
import frc.robot.subsystems.Swerve.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** A command that will turn the robot to the specified angle using a motion profile. */
public class StrafeToAprilTagProfiled extends ProfiledPIDCommand {
  
  private static ProfiledPIDController m_PID = new ProfiledPIDController(
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
        m_PID,
        // Close loop on heading
        aprilTags::getBestAprilTag3dAngle,
        // Set reference to target
        180,
        // Pipe output to turn robot
        (output, setpoint) -> drive.drive(0, -output, 0, false, true),
        // Require the drive
        drive);

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(DriveConstants.kYawToleranceDeg, DriveConstants.kYawRateToleranceDegPerS);
      
    // Add the PID to dashboard
    if (!m_shuffleboardLoaded) {
      ShuffleboardTab turnTab = Shuffleboard.getTab("Drivebase");
      turnTab.add("AprilTag PID4", m_PID);
      m_shuffleboardLoaded = true;  // so we do this only once no matter how many instances are created
    }
    m_aprilTags = aprilTags;
  
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    if (getController().atGoal()) {System.out.println("done strafing to April Tag");}
    return getController().atGoal();
  }

  @Override
  public void execute() {
    System.out.println("stafing to April Tag " + m_aprilTags.getBestAprilTag3dAngle());
    super.execute();
  }  
}
