// Author: UMN Robotics Ri3d
// Last Updated : January 2023

package frc.robot.Commands.DriveCommands;

import frc.robot.Constants.BalanceConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

// This command self=balances on the charging station using gyroscope pitch as feedback
public class Balance extends CommandBase {

  private DriveSubsystem m_DriveSubsystem;

  private double error;
  private double currentAngle;
  private double drivePower;

  /** Command to use Gyro data to resist the tip angle from the beam - to stabalize and balanace */
  public Balance() {
    this.m_DriveSubsystem = RobotContainer.m_robotDrive;
    addRequirements(m_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Uncomment the line below this to simulate the gyroscope axis with a controller joystick
    // Double currentAngle = -1 * Robot.controller.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS) * 45;
    this.currentAngle = m_DriveSubsystem.m_gyro.getPitch();

    error = BalanceConstants.BEAM_BALANCED_GOAL_DEGREES - currentAngle;
    drivePower = -Math.min(BalanceConstants.BEAM_BALANACED_DRIVE_KP * error, 1);

    // Our robot needed an extra push to drive up in reverse, probably due to weight imbalances
    if (drivePower < 0) {
      drivePower *= BalanceConstants.BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER;
    }

    // Limit the max power
    if (Math.abs(drivePower) > 0.4) {
      drivePower = Math.copySign(0.4, drivePower);
    }

    m_DriveSubsystem.drive(drivePower, 0, 0, false, true);
    
    // Debugging Print Statments
    System.out.println("Current Angle: " + currentAngle);
    System.out.println("Error " + error);
    System.out.println("Drive Power: " + drivePower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystem.setX();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(error) < BalanceConstants.BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES; // End the command when we are within the specified threshold of being 'flat' (gyroscope pitch of 0 degrees)
  }
}
