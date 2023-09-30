// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.EverybotConstants;
import frc.robot.subsystems.Arm.Everybot;

public class RunIntakeOut extends CommandBase {
  Everybot everybotIntake;
  /** Creates a new RunIntake. */
  public RunIntakeOut(Everybot intake) {
    everybotIntake = intake;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(everybotIntake.getMode() == "cone"){
      everybotIntake.setIntakeMotor(-1*EverybotConstants.INTAKE_OUTPUT_POWER, EverybotConstants.INTAKE_CURRENT_LIMIT_A);
    }
    else if (everybotIntake.getMode() == "cube"){
      everybotIntake.setIntakeMotor(EverybotConstants.INTAKE_OUTPUT_POWER, EverybotConstants.INTAKE_CURRENT_LIMIT_A);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
