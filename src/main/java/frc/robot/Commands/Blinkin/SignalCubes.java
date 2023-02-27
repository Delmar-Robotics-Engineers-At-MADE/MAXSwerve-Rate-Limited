// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Blinkin;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BlinkinSubsystem;

/** A command that will turn the robot to the specified angle. */
public class SignalCubes extends CommandBase {

  private BlinkinSubsystem m_lights;

  public SignalCubes(BlinkinSubsystem lights) {
    addRequirements(lights);
    m_lights = lights;
  }

  @Override
  public void execute() {
    m_lights.signalCubes();
  }

  @Override
  public boolean isFinished() {
    return true; //  run only while button is held down
  }
}
