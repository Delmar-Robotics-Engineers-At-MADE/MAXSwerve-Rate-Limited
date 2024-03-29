package frc.robot.Commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Claw;

public class HoldClawGrip extends CommandBase {

  private Claw m_claw;
  double m_targetPosition = 0.0;

  public HoldClawGrip(double position, Claw claw) {
    m_claw = claw;
    m_targetPosition = position;
    addRequirements(claw);
  }

  @Override
  public void execute() {
    m_claw.hold(m_targetPosition);
    // System.out.println("claw holding position " + m_targetPosition);
    super.execute();
  }

  @Override
  public boolean isFinished() {
      return m_claw.m_holding;  // run command once and it will continue until interrupted
                         // or m_holding gets cleared
  }

}
