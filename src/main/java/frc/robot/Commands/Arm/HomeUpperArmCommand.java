package frc.robot.Commands.Arm;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.UpperArmConstants;
import frc.robot.subsystems.Arm.UpperArmSubsystem;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** A command that will move the arm to home position using potentiometer or encoder */
public class HomeUpperArmCommand extends PIDCommand {

  private static PIDController m_PID = new PIDController(
    UpperArmConstants.kPotmeterP,  UpperArmConstants.kPotmeterI, UpperArmConstants.kPotmeterD);

    private static boolean m_shuffleboardLoaded = false;
    private static UpperArmSubsystem m_upperArm;

  public HomeUpperArmCommand(UpperArmSubsystem upperArm) {
    super(
      m_PID,  
        upperArm::potPosition, // Close loop on position
        UpperArmConstants.kHomePotmeterValue,  // Set setpoint to target distance
        output -> upperArm.moveOpenLoop(-output), // Pipe output to hood to elevate
        upperArm);  // Require the subsystem
        m_upperArm = upperArm;

    PIDController pid = getController();
    pid.setTolerance(UpperArmConstants.kPotmeterTolerance);
      
    // Add the PID to dashboard
    if (!m_shuffleboardLoaded) {
      ShuffleboardTab dashboardTab = Shuffleboard.getTab("Arm");
      dashboardTab.add("Homing PID", m_PID);
      m_shuffleboardLoaded = true; // so we do this only once for the class
    }
  }

  // @Override
  // public void execute() {
  //   // TODO Auto-generated method stub
  //   super.execute();
  //   if ()
  // }

  @Override
  public boolean isFinished() {
    boolean result = getController().atSetpoint();
    if (result) {
      m_upperArm.setEncoderHomed();
    }
    return result;
  }
}
