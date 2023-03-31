package frc.robot.Commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Claw;

/* Use this command like so:
 *
 *  private final SequentialCommandGroup m_moveAndHoldCommand= new SequentialCommandGroup(
    new MoveClawUntilStall(CLAW_CONSTANTS.kInVelocity, m_claw), 
    new PrepareToHold(m_claw),
    new HoldClawGrip(0.0, m_claw)
  );

    new JoystickButton(m_driverController, Button.kA.value)
      .toggleOnTrue(m_moveAndHoldCommand);

 */
public class MoveClawUntilStall extends CommandBase {

  private Claw m_claw;
  double m_targetSpeed = 0.0;

  public MoveClawUntilStall(double speed, Claw claw) {
    addRequirements(claw);
    m_claw = claw;
    m_targetSpeed = speed;
  }

  @Override
  public void execute() {
    m_claw.runClawClosedLoop(m_targetSpeed); // testing on celestial was with 10000 to 30000
    super.execute();
  }

  @Override
  public boolean isFinished() {
    // finish when stalled, so we can transition to holding
    // System.out.println("Stalled " + m_claw.checkStalledCondition());
    if (m_claw.checkStalledCondition()) {
      return true;
    } else {
      return false; // run this command only once, and it will run until stalled
    } 
  }

}
