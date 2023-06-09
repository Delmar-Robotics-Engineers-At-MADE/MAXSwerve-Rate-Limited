package frc.robot.Commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SWORD_CONSTANTS;
import frc.robot.subsystems.Arm.Lightsaber;

public class SwordToPosition extends CommandBase {

  private double m_target;
  private Lightsaber m_sword;

  public SwordToPosition(double target, Lightsaber sword) {
    m_target = target;
    m_sword = sword;
    addRequirements(sword);
  }

  @Override
  public void initialize() {
    super.initialize();
  }

  @Override
  public void execute() {
    m_sword.hold(m_target);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_sword.runSwordClosedLoop(0);
  }

  @Override
  public boolean isFinished() {
    return m_sword.atClosedLoopTarget();
    // return Math.abs(m_sword.getClosedLoopSensorValue() - m_target) < SWORD_CONSTANTS.kPIDPositionTolerance;
    // m_sword.get
  }

}
