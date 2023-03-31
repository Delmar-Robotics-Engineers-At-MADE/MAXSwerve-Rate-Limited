// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.DriveCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Swerve.DriveSubsystem;
import frc.robot.subsystems.Cameras.LimelightSubsystem;
import frc.robot.subsystems.Arm.Claw;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


/** A command that will turn the robot to the specified angle using a motion profile. */
public class MoveToGamepieceProfiled extends ProfiledPIDCommand {
  
  private Claw m_claw;

  private static ProfiledPIDController m_PID = new ProfiledPIDController(
    DriveConstants.kYawP, DriveConstants.kYawI, DriveConstants.kYawD,
    new TrapezoidProfile.Constraints(
                DriveConstants.kMaxYawRateDegPerS,
                DriveConstants.kMaxYawAccelerationDegPerSSquared));

  private static boolean m_shuffleboardLoaded = false;
  /**
   * Turns to robot to the specified angle using a motion profile.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
  public MoveToGamepieceProfiled(double forwardSpeed,
                  LimelightSubsystem limelight, DriveSubsystem drive, Claw claw) {
    super(
        m_PID,
        // Close loop on heading
        limelight::getBestGamepieceYaw,
        // Set reference to target
        CameraConstants.kGamepieceCenterPos,
        // Pipe output to turn robot
        (output, setpoint) -> drive.drive(forwardSpeed, 0, output, false, true),
        // Require the drive
        drive);

    m_claw = claw;

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(DriveConstants.kMaxYawRateDegPerS, DriveConstants.kMaxYawAccelerationDegPerSSquared);
      
        // Add the PID to dashboard
      if (!m_shuffleboardLoaded) {
        ShuffleboardTab turnTab = Shuffleboard.getTab("Drivebase");
        turnTab.add("Gamepiece PID 2", m_PID);
        turnTab.addDouble("Gamepiece error 2", () -> m_PID.getPositionError());
        m_shuffleboardLoaded = true;  // so we do this only once no matter how many instances are created
      }
      System.out.println("new turn to limelight command created");
  
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    // return getController().atGoal();
    // End when a gamepiece is collected
    return m_claw.m_holding;
  }
}
