// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Cameras.LimelightSubsystem;
import frc.robot.Constants.CameraConstants;

/** A command that will turn the robot to the specified angle. */
public class UpdateBestGamepieceCommand extends CommandBase {

  private LimelightSubsystem m_limelight;

  public UpdateBestGamepieceCommand(LimelightSubsystem limelight) {
    m_limelight = limelight;
  }

  @Override
  public void execute() {
    m_limelight.updateBestLimelight(CameraConstants.kGamepieceCenterPos);
  }

  // @Override
  // public boolean isFinished() {
  //   return true; //  run only while button is held down
  // }
}
