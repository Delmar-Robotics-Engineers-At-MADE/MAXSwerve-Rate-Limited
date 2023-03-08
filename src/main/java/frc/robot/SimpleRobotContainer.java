// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Commands.Arm.MoveUpperArmCommand;
// import frc.robot.Commands.Blinkin.DefaultLighting;
// import frc.robot.Commands.DriveCommands.Balance;
// import frc.robot.Commands.DriveCommands.DriveToAprilTag;
// import frc.robot.Commands.DriveCommands.TurnToAprilTagProfiled;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.UpperArmConstants;
import frc.robot.Constants.ControllerConstants.DriverConstants;
import frc.robot.Constants.ControllerConstants.OpperatorConstants;
// import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.subsystems.Arm.LowerArm;
import frc.robot.subsystems.Arm.UpperArmSubsystem;
// import frc.robot.subsystems.Cameras.AprilTagSubsystem;
// import frc.robot.subsystems.Cameras.LimelightSubsystem;
// import frc.robot.subsystems.Swerve.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import frc.robot.Commands.Arm.HomeUpperArmCommand;
// import frc.robot.Commands.Arm.MoveUpperArmCommand;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class SimpleRobotContainer {
// The robot's subsystems
  private static final Claw m_claw = new Claw();
  private static final UpperArmSubsystem m_upperArm = new UpperArmSubsystem();
  private static final LowerArm m_lowerArm = new LowerArm();
  

  // The driver's controller
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  JoystickButton m_turoButton = new JoystickButton(m_driverController, DriverConstants.TURBO);
  JoystickButton m_crawlButton = new JoystickButton(m_driverController, DriverConstants.CRAWL);
  JoystickButton m_autoBalance = new JoystickButton(m_driverController, DriverConstants.kAutoBalance);

  //The opperator's controller
  Joystick m_opperator = new Joystick(OIConstants.kOpperatorControllerPort);
  JoystickButton m_ODrive = new JoystickButton(m_opperator, OpperatorConstants.PRIORITY_LEFT);
  JoystickButton m_OSlow = new JoystickButton(m_opperator, OpperatorConstants.kOSlow);
  Trigger m_lowerArmUp = new JoystickButton(m_opperator, 5);
  Trigger m_lowerArmDown = new JoystickButton(m_opperator, 6);

  private ShuffleboardTab m_comp;

  private void configureButtonBindings() {
    // new JoystickButton(m_driverController, DriverConstants.X_MODE)
    //     .toggleOnTrue(new RunCommand(
    //         () -> m_robotDrive.setX(),
    //         m_robotDrive));
    
    // new JoystickButton(m_driverController, DriverConstants.ZERO_HEADING)
    //     .whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
    
    // new JoystickButton(m_driverController, DriverConstants.kAutoBalance)
    //     .toggleOnTrue(m_balance);

    new JoystickButton(m_opperator, OpperatorConstants.kNudgeUp)
    .toggleOnTrue(new InstantCommand(
        () -> m_upperArm.nudgeClosedLoopByFalconEnc(true), m_upperArm ));

    new JoystickButton(m_opperator, OpperatorConstants.kNudgeDown)
    .toggleOnTrue(new InstantCommand(
        () -> m_upperArm.nudgeClosedLoopByFalconEnc(false), m_upperArm ));
    
    // new JoystickButton(m_opperator, OpperatorConstants.kMoveUpperArmToTarget)
    // .toggleOnTrue(new MoveUpperArmCommand(200, m_upperArm));

    // new JoystickButton(m_driverController, DriverConstants.kTurnToTag)
    // .toggleOnTrue(new TurnToAprilTagProfiled(0, m_aprilTags, m_robotDrive));

    // new JoystickButton(m_driverController, DriverConstants.kDriveToTag)
    // .toggleOnTrue(new DriveToAprilTag(0.5, m_aprilTags, m_robotDrive));

    new JoystickButton(m_opperator, OpperatorConstants.kFloorMode)
    .whileTrue(m_lowerArm.lowerArmFloorPosition());

    new JoystickButton(m_opperator, OpperatorConstants.kHighPosition)
    .whileTrue(m_lowerArm.lowerArmHighPosition());
    new JoystickButton(m_opperator, OpperatorConstants.kHighPosition)
    .whileTrue(new MoveUpperArmCommand(UpperArmConstants.kHighPosition, m_upperArm));

    new JoystickButton(m_driverController, DriverConstants.kMidPosition)
    .whileTrue(m_lowerArm.lowerArmMidPosition());
    new JoystickButton(m_driverController, DriverConstants.kMidPosition)
    .whileTrue(new MoveUpperArmCommand(UpperArmConstants.kMidPosition, m_upperArm));

    new JoystickButton(m_opperator, OpperatorConstants.kShootCubeHigh)
    .whileTrue(m_lowerArm.lowerArmShootPosition());

    new JoystickButton(m_opperator, DriverConstants.kSingleSubstation)
    .whileTrue(m_lowerArm.lowerArmSSsPosition());

    new JoystickButton(m_opperator, OpperatorConstants.kHomeArms)
    .whileTrue(m_lowerArm.homeLowerArm());
    new JoystickButton(m_opperator, OpperatorConstants.kHomeArms)
    .whileTrue(new HomeUpperArmCommand(m_upperArm));

    new JoystickButton(m_driverController, DriverConstants.kLowerArmUp)
    .whileTrue(m_lowerArm.runLowerArmUp());

    new JoystickButton(m_driverController, DriverConstants.kLowerArmDown)
    .whileTrue(m_lowerArm.runLowerArmDown());

    new JoystickButton(m_driverController, 9)
    .whileTrue(new HomeUpperArmCommand(m_upperArm));
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public SimpleRobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_comp = Shuffleboard.getTab("Competition");

    // Configure default commands
    CommandScheduler.getInstance().setDefaultCommand(m_claw, m_claw.stop());


    // Why can't we specify dependency on subsystem here???

    m_lowerArm.setDefaultCommand(
      m_lowerArm.lowerArmHoldPosition());
    
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // return something bogus
    return new HomeUpperArmCommand(m_upperArm);
  }
}
