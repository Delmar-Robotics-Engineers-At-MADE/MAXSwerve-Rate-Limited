// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Cameras.AprilTagSubsystem;
import frc.robot.subsystems.Swerve.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** A command that will turn the robot to the specified angle using a motion profile. */
public class Balance2 extends PIDCommand {
  
  private static PIDController m_PID = new PIDController(
    DriveConstants.kDriveP, DriveConstants.kDriveI, DriveConstants.kDriveD);

  DriveSubsystem m_robotDrive;

  private static boolean m_shuffleboardLoaded = false;
  /**
   * Turns to robot to the specified angle using a motion profile.
   *
   * @param targetDistance The angle to turn to
   * @param drive The drive subsystem to use
   */
  public Balance2(double targetDistance, DriveSubsystem drive) {
    super(
        m_PID,
        // Close loop on heading
        drive::getPitch,
        // Set reference to target
        targetDistance,
        // Pipe output to turn robot
        output -> drive.drive(-output, 0, 0, false, true),
        // Require the drive
        drive);

        m_robotDrive = drive;
     getController().setTolerance(1);
      
    // Add the PID to dashboard
    if (!m_shuffleboardLoaded) {
      ShuffleboardTab turnTab = Shuffleboard.getTab("Drivebase");
      turnTab.add("Balance2 PID", m_PID);
      m_shuffleboardLoaded = true;  // so we do this only once no matter how many instances are created
    }
  
  }
  @Override
  public void execute() {
    System.out.println(m_PID.getVelocityError());
    super.execute();
  }

  @Override
  public void end(boolean intterupted) {
    m_robotDrive.setX();
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }
}
