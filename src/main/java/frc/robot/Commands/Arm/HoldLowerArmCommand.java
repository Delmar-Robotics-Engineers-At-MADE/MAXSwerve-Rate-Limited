package frc.robot.Commands.Arm;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.UpperArmConstants;
import frc.robot.subsystems.Arm.LowerArm;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** A command that will move the arm to home position using potentiometer or encoder */
public class HoldLowerArmCommand extends CommandBase {

    private static boolean m_shuffleboardLoaded = false;
    private static LowerArm m_lowerArm;

  public HoldLowerArmCommand(LowerArm lowerArm) {
    m_lowerArm = lowerArm;
    addRequirements(lowerArm);
  }

  @Override
  public void execute() {
    m_lowerArm.holdLowerArm();
  }

  @Override
  public boolean isFinished() {
    return true; // run over and orver again
  }
}
