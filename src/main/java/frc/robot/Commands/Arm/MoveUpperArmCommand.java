package frc.robot.Commands.Arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.UpperArmConstants;
import frc.robot.subsystems.Arm.UpperArmSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** A command that will move the arm to a specified Rev encoder position */
public class MoveUpperArmCommand extends ProfiledPIDCommand {

  // private static PIDController m_PID = new PIDController(
  //   UpperArmConstants.kRevEncoderP,  UpperArmConstants.kRevEncoderI, UpperArmConstants.kRevEncoderD);

  private static ProfiledPIDController m_PID = new ProfiledPIDController(
    UpperArmConstants.kRevEncoderP,  
    UpperArmConstants.kRevEncoderI, 
    UpperArmConstants.kRevEncoderD,
      new TrapezoidProfile.Constraints(
        UpperArmConstants.kRevEncoderMaxCountsPerS,
        UpperArmConstants.kRevEncoderMaxCountsPerSSquared));

  private UpperArmSubsystem m_upperArm;
  private static boolean m_shuffleboardLoaded = false;

  public MoveUpperArmCommand(double target, UpperArmSubsystem upperArm) {
    super(
        m_PID,  
        upperArm::encoderPosition, // Close loop on position
        target,  // Set setpoint to target distance
        (output, setpoint) -> upperArm.moveOpenLoop(output), // Pipe output to hood to elevate
        upperArm);  // Require the subsystem

    m_upperArm = upperArm;
    getController().setTolerance(UpperArmConstants.kRevEncoderTolerance,
                                 UpperArmConstants.kRevEncoderMaxCountsPerSTolerance);
      
    // Add the PID to dashboard
    if (!m_shuffleboardLoaded) {
      ShuffleboardTab dashboardTab = Shuffleboard.getTab("Arm");
      dashboardTab.add("Moving PID", m_PID);
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
    if (!m_upperArm.m_encoderHomed) {
      // don't use this command until homed
      return true;
    } else {
      return getController().atGoal();
    }
  }
}
