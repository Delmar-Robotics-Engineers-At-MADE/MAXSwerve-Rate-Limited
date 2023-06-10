// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SWORD_CONSTANTS;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.Arm.Lightsaber;
import frc.robot.subsystems.Cameras.LimelightSubsystem;

public class SwordFight extends CommandBase {

  private static final double kLightsaberCenterRange = 10;

  private LimelightSubsystem m_limelight;
  private Lightsaber m_sword;
  private BlinkinSubsystem m_lights;

  public SwordFight(Lightsaber sword, LimelightSubsystem limelight, BlinkinSubsystem lights) {
    addRequirements(lights);
    addRequirements(sword);
    addRequirements(limelight);
    m_sword = sword;
    m_limelight = limelight;
    m_lights = lights;
    m_limelight.turnLightOnOrOff(false);
  }

  @Override
  public void execute() {
    if (m_limelight.getInstantaneousYaw() < -kLightsaberCenterRange) {
      m_lights.signalA();
      m_sword.hold(SWORD_CONSTANTS.kParryLeft);
    } else if (m_limelight.getInstantaneousYaw() > kLightsaberCenterRange) {
      m_lights.signalB();
      m_sword.hold(SWORD_CONSTANTS.kParryRight);
    } else {
      m_lights.defaultLighting();
      m_sword.hold(0);
    }
  }

  @Override
  public boolean isFinished() {
    return false; //  run until interrupted
  }
}
