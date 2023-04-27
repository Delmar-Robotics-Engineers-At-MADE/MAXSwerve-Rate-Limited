package frc.robot.Commands.Arm;

import frc.robot.Constants.LowerArmConstants;
import frc.robot.subsystems.Arm.LowerArm;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** A command that will move the arm to home position using potentiometer or encoder */
public class MoveLowerArmCommand extends CommandBase {

  private LowerArm m_lowerArm;
  private double m_target;
  

  public MoveLowerArmCommand(double pos, LowerArm lowerArm) {
    m_target = pos;
    m_lowerArm = lowerArm;
    addRequirements(lowerArm);
    System.out.println("lower arm command taget: " + m_target);
  }

  @Override
  public void execute() {
    m_lowerArm.runLowerArmClosedLoop(m_target);
  }

  @Override
  public boolean isFinished() {
    boolean result = Math.abs(m_lowerArm.getElbowPos() - m_target) < LowerArmConstants.kTolerance;
    if (result) System.out.println("Lower arm at target: " + m_target + " = " + m_lowerArm.getElbowPos());
    return (result);
  }
}
