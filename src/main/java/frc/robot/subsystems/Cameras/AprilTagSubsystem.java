// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Cameras;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Rotation3d;


public class AprilTagSubsystem extends SubsystemBase {

  private PhotonCamera m_photonCamera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
  private PhotonPipelineResult m_latestPhotonResult;
  private double m_bestAprilTagYaw = 0.0;
  private int    m_bestAprilTagID = 0;
  private double m_bestAprilTagPitch=0.0;
  private double m_bestAprilTagDistance = 0.0;
  private double m_bestAprilTag3dX = 0.0;
  private double m_bestAprilTag3dAngle = 0.0;

  public AprilTagSubsystem() {
    ShuffleboardTab driveBaseTab = Shuffleboard.getTab("Cameras");
    driveBaseTab.addDouble("Tag Yaw", () -> m_bestAprilTagYaw);
    driveBaseTab.addDouble("Tag ID", () -> m_bestAprilTagID);
    driveBaseTab.addDouble("Tag Distance", () -> m_bestAprilTagDistance);
    driveBaseTab.addDouble("Tag 3d X", () -> m_bestAprilTag3dX);
    driveBaseTab.addDouble("Tag 3d angle", () -> m_bestAprilTag3dAngle);
    // driveBaseTab.addDouble("Tag Skew", () -> m_bestAprilTagSkew);
  }


  // call this from PID command to turn robot to best target
  public double getBestAprilTagYaw() {
    updateBestAprilTag();
    return m_bestAprilTagYaw;
  }

  // call this from PID command when searching for unseen tag
  public double getPhantomAprilTagYaw() {
    updateBestAprilTag();
    if (m_bestAprilTagYaw == 0.0) {
      // no april tag seen, so return a yaw that will cause robot to rotate
      return CameraConstants.kSummerSearchForAprilTagYaw;
    } else {
      // tag seen
      return m_bestAprilTagYaw;
    }
  }

  public double getBestAprilTagPitch() {
    updateBestAprilTag();
    return m_bestAprilTagPitch;
  }

  public double getBestAprilTag3dAngle() {
    updateBestAprilTag();
    return m_bestAprilTag3dAngle;
  }

  public double getBestAprilTag3dX() {
    updateBestAprilTag();
    return m_bestAprilTag3dX;
  }

  public double getBestAprilTagDistance() {
    updateBestAprilTag();
    return m_bestAprilTagDistance;
  }

  public void updateBestAprilTag() {
    m_latestPhotonResult = m_photonCamera.getLatestResult();
    if (m_latestPhotonResult.hasTargets()) {
      PhotonTrackedTarget bestTarget = m_latestPhotonResult.getBestTarget();
      m_bestAprilTagYaw = bestTarget.getYaw();
      m_bestAprilTagPitch = bestTarget.getPitch();
      m_bestAprilTagID = bestTarget.getFiducialId();
      // m_bestAprilTagSkew = bestTarget.getSkew();
      Transform3d bestCameraToTarget = m_latestPhotonResult.getBestTarget().getBestCameraToTarget();
      m_bestAprilTagDistance = PhotonUtils.calculateDistanceToTargetMeters(
          CameraConstants.CAMERA_HEIGHT_METERS,
          CameraConstants.TARGET_HEIGHT_METERS,
          CameraConstants.CAMERA_PITCH_RADIANS,
          Units.degreesToRadians(m_latestPhotonResult.getBestTarget().getPitch()));
      m_bestAprilTag3dX = bestCameraToTarget.getX();
      Rotation3d rotation = bestCameraToTarget.getRotation();
      m_bestAprilTag3dAngle = Units.radiansToDegrees(rotation.getAngle());
    } else {
      m_bestAprilTagYaw = 0.0;
      m_bestAprilTagPitch = 0.0;
      m_bestAprilTagID = 0;
      m_bestAprilTagDistance =0;
      m_bestAprilTag3dX = 0;
      m_bestAprilTag3dAngle = 180;
    }
  }
  
}






