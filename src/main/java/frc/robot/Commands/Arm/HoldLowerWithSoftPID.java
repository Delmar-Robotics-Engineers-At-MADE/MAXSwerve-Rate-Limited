package frc.robot.Commands.Arm;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.LowerArmConstants;
import frc.robot.subsystems.Arm.LowerArm;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class HoldLowerWithSoftPID extends PIDCommand {

  private static PIDController m_PID = new PIDController(
    LowerArmConstants.kP,  LowerArmConstants.kI, LowerArmConstants.kD);

    private static boolean m_shuffleboardLoaded = false;
    // private static LowerArm m_lowerArm;

  public HoldLowerWithSoftPID(double target, LowerArm lowerArm) {
    super(
      m_PID,  
      lowerArm::getElbowPos, // Close loop on position
      target,  // Set setpoint to target distance
      output -> lowerArm.runlowerArmOpenLoop(output), // Pipe output to motor
      lowerArm);  // Require the subsystem
      // m_lowerArm = lowerArm;

    PIDController pid = getController();
    pid.setTolerance(LowerArmConstants.kTolerance);
      
    // Add the PID to dashboard
    if (!m_shuffleboardLoaded) {
      ShuffleboardTab dashboardTab = Shuffleboard.getTab("Arm");
      dashboardTab.add("Elbow soft PID", m_PID);
      dashboardTab.addDouble("Elbow error", () -> m_PID.getPositionError());
      m_shuffleboardLoaded = true; // so we do this only once for the class
    }
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
