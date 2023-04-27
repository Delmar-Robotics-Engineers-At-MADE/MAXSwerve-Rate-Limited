package frc.robot.Commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.Constants.CLAW_CONSTANTS;

public class ShootCubeTimed extends CommandBase {

  private Claw m_claw;
  private long m_startTime;
  private final double m_duration;

  public ShootCubeTimed(double timeSecs, Claw claw) {
    m_duration = timeSecs * 1000;
    m_claw = claw;
    // do not add requirement so can use in summer sequence, was...
    //   addRequirements(claw);
  }

  @Override
  public void initialize() {
    m_startTime = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    m_claw.runClawClosedLoop(CLAW_CONSTANTS.kCubeOutVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_claw.runClawClosedLoop(0);
  }

  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - m_startTime) >= m_duration;
  }

}
