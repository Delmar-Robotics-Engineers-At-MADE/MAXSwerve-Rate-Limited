// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Claw;

/** A command that will turn the robot to the specified angle. */
public class PrepareToHold extends CommandBase {

  private Claw m_claw;

  public PrepareToHold(Claw claw) {
    m_claw = claw;
    addRequirements(claw);
  }

  @Override
  public void execute() {
    m_claw.prepareToHold();
    super.execute();
  }

  @Override
  public boolean isFinished() {
      return true;  
  }

}
