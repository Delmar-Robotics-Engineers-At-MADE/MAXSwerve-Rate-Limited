// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Everybot;

public class RunIntake extends CommandBase {
  Everybot everybotIntake;
  boolean isReverse;
  double speed;
  int amps;
  /** Creates a new RunIntake. */
  public RunIntake(Everybot intake, boolean reversed, double percent, int current) {
    everybotIntake = intake;
    isReverse = reversed;
    speed = percent;
    amps = current;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isReverse) {
      everybotIntake.setIntakeMotor(-1*speed, amps);
    }
    else {
      everybotIntake.setIntakeMotor(speed, amps);
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
