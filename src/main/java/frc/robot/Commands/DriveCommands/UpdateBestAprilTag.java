// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Cameras.AprilTagSubsystem;

/** A command that will turn the robot to the specified angle. */
public class UpdateBestAprilTag extends CommandBase {

  private AprilTagSubsystem m_aprilTags;

  public UpdateBestAprilTag(AprilTagSubsystem aprilTags) {
    m_aprilTags = aprilTags;
  }

  @Override
  public void execute() {
    m_aprilTags.updateBestAprilTag();
  }

  // @Override
  // public boolean isFinished() {
  //   return true; //  run only while button is held down
  // }
}
