// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.Arm.Everybot;

public class SetIntakeMode extends CommandBase {
  Everybot everybotIntake;
  BlinkinSubsystem lights;
  String mode;
  /** Creates a new SetIntakeMode. */
  public SetIntakeMode(String inMode, Everybot intake, BlinkinSubsystem blinkin) {
    everybotIntake = intake;
    lights = blinkin;
    mode = inMode;
    addRequirements(intake, blinkin);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    everybotIntake.setMode(mode);
    if(everybotIntake.getMode()=="cone"){
      lights.signalCones();
    }
    else{
      lights.signalCubes();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
