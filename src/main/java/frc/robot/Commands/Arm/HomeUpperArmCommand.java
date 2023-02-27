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

  /**
   * Raises hood to a specified distance.
   *
   * @param targetDistance The encoder distance to go to
   * @param hood The subsystem to use
   */
  public HomeUpperArmCommand(UpperArmSubsystem upperArm) {
    super(
      m_PID,  
        upperArm::potPosition, // Close loop on position
        UpperArmConstants.kHomePotmeterValue,  // Set setpoint to target distance
        output -> upperArm.moveOpenLoop(output), // Pipe output to hood to elevate
        upperArm);  // Require the subsystem

    PIDController pid = getController();
    if (upperArm.m_encoderHomed) {
      // if already homed by potentiometer, then can return to home using encoder
      pid.setP(UpperArmConstants.kRevEncoderP);
      pid.setI(UpperArmConstants.kRevEncoderI);
      pid.setD(UpperArmConstants.kRevEncoderD);
      pid.setTolerance(UpperArmConstants.kPotmeterTolerance);
      pid.setSetpoint(UpperArmConstants.kHomeEncoderValue);
    } else { // continue with potentiometer homing
      pid.setTolerance(UpperArmConstants.kPotmeterTolerance);
    }
      
    // Add the PID to dashboard
    if (!m_shuffleboardLoaded) {
      ShuffleboardTab dashboardTab = Shuffleboard.getTab("Arm");
      dashboardTab.add("Homing PID", m_PID);
      m_shuffleboardLoaded = true; // so we do this only once for the class
    }
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    // System.out.println(getController().getPositionError());
    // boolean result = Math.abs(getController().getPositionError()) 
    //   <= DriveConstants.kTurnToleranceDeg;
    // return result;
    return getController().atSetpoint();
  }
}
