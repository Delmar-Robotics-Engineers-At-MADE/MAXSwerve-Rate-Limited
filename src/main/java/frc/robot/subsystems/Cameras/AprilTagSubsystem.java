// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Cameras;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;


public class AprilTagSubsystem extends SubsystemBase {

  private PhotonCamera m_photonCamera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
  private PhotonPipelineResult m_latestPhotonResult;
  private double m_bestAprilTagYaw = 0.0;
  private int m_bestAprilTagID = 0;
  private double m_bestAprilTagPitch=0.0;
  private double  m_bestAprilTagDistance = 0.0;

  public AprilTagSubsystem() {
    ShuffleboardTab driveBaseTab = Shuffleboard.getTab("Cameras");
    driveBaseTab.addDouble("Tag Yaw", () -> m_bestAprilTagYaw);
    driveBaseTab.addDouble("Tag ID", () -> m_bestAprilTagID);
    driveBaseTab.addDouble("Tag Distance", () -> m_bestAprilTagDistance);
  }


  // call this from PID command to turn robot to best target
  public double getBestAprilTagYaw() {
    updateBestAprilTag();
    return m_bestAprilTagYaw;
  }

  public double getBestAprilTagPitch() {
    updateBestAprilTag();
    return m_bestAprilTagPitch;
  }

  public double getBestAprilTagDistance() {
    updateBestAprilTag();
    return m_bestAprilTagDistance;
  }

  public void updateBestAprilTag() {
    m_latestPhotonResult = m_photonCamera.getLatestResult();
    if (m_latestPhotonResult.hasTargets()) {
      m_bestAprilTagYaw = m_latestPhotonResult.getBestTarget().getYaw();
      m_bestAprilTagPitch = m_latestPhotonResult.getBestTarget().getPitch();
      m_bestAprilTagID = m_latestPhotonResult.getBestTarget().getFiducialId();
      m_bestAprilTagDistance = PhotonUtils.calculateDistanceToTargetMeters(
          DriveConstants.CAMERA_HEIGHT_METERS,
          DriveConstants.TARGET_HEIGHT_METERS,
          DriveConstants.CAMERA_PITCH_RADIANS,
          Units.degreesToRadians(m_latestPhotonResult.getBestTarget().getPitch()));
    } else {
      m_bestAprilTagYaw = 0.0;
      m_bestAprilTagPitch = 0.0;
      m_bestAprilTagID = 0;
      m_bestAprilTagDistance =0;
    }
  }
  
}






