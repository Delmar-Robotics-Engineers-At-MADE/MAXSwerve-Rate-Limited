// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.SWORD_CONSTANTS;
import frc.robot.subsystems.Arm.Lightsaber;
import frc.robot.subsystems.Cameras.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class SwordHomeByLimelight extends ProfiledPIDCommand {
  
  private LimelightSubsystem m_limelight;
  private Lightsaber m_sword;

  private static ProfiledPIDController m_PID = new ProfiledPIDController(
    SWORD_CONSTANTS.kYawP, SWORD_CONSTANTS.kYawI, SWORD_CONSTANTS.kYawD,
    new TrapezoidProfile.Constraints(
      SWORD_CONSTANTS.kMaxYawRateDegPerS,
      SWORD_CONSTANTS.kMaxYawAccelerationDegPerSSquared));

  // private static boolean m_shuffleboardLoaded = false;

  public SwordHomeByLimelight(LimelightSubsystem limelight, Lightsaber sword) {
    super(
        m_PID,
        // Close loop on heading
        limelight::getBestLimelightYaw,
        // Set reference to target
        -9,
        // Pipe output to move lightsaber
        (output, setpoint) ->  sword.runSwordOpenLoop(-output),
        // Require the drive
        sword);

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(SWORD_CONSTANTS.kYawToleranceDeg, SWORD_CONSTANTS.kYawRateToleranceDegPerS);
      
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
    boolean result = getController().atGoal();
    if (result == true) {m_sword.setEncoderHomed();}
    return result;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_limelight.turnLightOnOrOff(false);
    super.end(interrupted);
  }  

}
