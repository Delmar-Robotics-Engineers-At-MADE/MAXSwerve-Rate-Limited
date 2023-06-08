// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Blinkin;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.Cameras.LimelightSubsystem;

public class SignalLightSaber extends CommandBase {

  private static final double kLightsaberCenterRange = 10;

  private BlinkinSubsystem m_lights;
  private LimelightSubsystem m_limelight;

  public SignalLightSaber(LimelightSubsystem limelight, BlinkinSubsystem lights) {
    addRequirements(lights);
    addRequirements(limelight);
    m_lights = lights;
    m_limelight = limelight;
    m_limelight.turnLightOnOrOff(false);
  }

  @Override
  public void execute() {
    if (m_limelight.getInstantaneousYaw() < -kLightsaberCenterRange) {
      m_lights.signalA();
    } else if (m_limelight.getInstantaneousYaw() > kLightsaberCenterRange) {
      m_lights.signalB();
    } else {
      m_lights.defaultLighting();
    }
  }

  @Override
  public boolean isFinished() {
    return true; //  run only while button is held down
  }
}
