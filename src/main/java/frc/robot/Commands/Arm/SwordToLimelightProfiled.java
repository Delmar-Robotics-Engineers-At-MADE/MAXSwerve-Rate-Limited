// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Arm.Lightsaber;
import frc.robot.subsystems.Cameras.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class SwordToLimelightProfiled extends ProfiledPIDCommand {
  
  private LimelightSubsystem m_limelight;
  private Lightsaber m_sword;

  private static ProfiledPIDController m_PID = new ProfiledPIDController(
    DriveConstants.kYawP, DriveConstants.kYawI, DriveConstants.kYawD,
    new TrapezoidProfile.Constraints(
                DriveConstants.kMaxYawRateDegPerS,
                DriveConstants.kMaxYawAccelerationDegPerSSquared));

  // private static boolean m_shuffleboardLoaded = false;

  public SwordToLimelightProfiled(LimelightSubsystem limelight, Lightsaber sword) {
    super(
        m_PID,
        // Close loop on heading
        limelight::getBestLimelightYaw,
        // Set reference to target
        -30,
        // Pipe output to move lightsaber
        (output, setpoint) ->  sword.runSwordOpenLoop(-output),
        // Require the drive
        sword);

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(DriveConstants.kMaxYawRateDegPerS, DriveConstants.kMaxYawAccelerationDegPerSSquared);
      
        // Add the PID to dashboard
      // if (!m_shuffleboardLoaded) {
      //   ShuffleboardTab turnTab = Shuffleboard.getTab("Drivebase");
      //   turnTab.add("Limelight PID", m_PID);
      //   m_shuffleboardLoaded = true;  // so we do this only once no matter how many instances are created
      // }

    m_limelight = limelight;
    m_sword = sword;
  
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atGoal();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_limelight.turnLightOnOrOff(false);
    m_sword.runSwordOpenLoop(0);
    m_sword.setEncoderHomed();
    super.end(interrupted);
  }  

}
