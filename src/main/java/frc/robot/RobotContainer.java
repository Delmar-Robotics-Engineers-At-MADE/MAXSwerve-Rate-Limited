// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ControllerConstants.DriverConstants;
import frc.robot.Constants.ControllerConstants.OpperatorConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.subsystems.Arm.LowerArm;
import frc.robot.subsystems.Swerve.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import java.util.List;
import java.util.concurrent.TransferQueue;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static DriveSubsystem m_drobotDrive;
// The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Claw m_claw = new Claw();
  private final LowerArm m_LowerArm = new LowerArm();

  // The driver's controller
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  JoystickButton m_turoButton = new JoystickButton(m_driverController, DriverConstants.TURBO);
  JoystickButton m_crawlButton = new JoystickButton(m_driverController, DriverConstants.CRAWL);

  //The opperator's controller
  Joystick m_opperator = new Joystick(OIConstants.kOpperatorControllerPort);
  JoystickButton m_ODrive = new JoystickButton(m_opperator, OpperatorConstants.PRIORITY_LEFT);
  JoystickButton m_OSlow = new JoystickButton(m_opperator, OpperatorConstants.kOSlow);
  JoystickButton m_clawTest = new JoystickButton(m_opperator, OpperatorConstants.kClawTest);
  POVButton m_lowerArmUp = new POVButton(m_opperator, 0);
  POVButton m_lowerArmDown = new POVButton(m_opperator, 180);

  //Sendable Chooser

  
   /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, DriverConstants.X_MODE)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    
    new JoystickButton(m_driverController, DriverConstants.ZERO_HEADING)
        .whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband((DriveConstants.kNormalSpeed * m_driverController.getRawAxis(1)), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband((DriveConstants.kNormalSpeed * m_driverController.getRawAxis(0)), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(DriveConstants.kNormalSpeed * m_driverController.getRawAxis(2), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

    m_turoButton.whileTrue(new RunCommand(
        () -> m_robotDrive.drive(
        -MathUtil.applyDeadband((DriveConstants.kTurboSpeed * m_driverController.getRawAxis(1)), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband((DriveConstants.kTurboSpeed * m_driverController.getRawAxis(0)), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(DriveConstants.kTurboSpeed * m_driverController.getRawAxis(2), OIConstants.kDriveDeadband),
        true, true),
    m_robotDrive));

    m_crawlButton.whileTrue(new RunCommand(
        () -> m_robotDrive.drive(
        -MathUtil.applyDeadband((DriveConstants.kCrawlSpeed * m_driverController.getRawAxis(1)), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband((DriveConstants.kCrawlSpeed * m_driverController.getRawAxis(0)), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(DriveConstants.kCrawlSpeed * (m_driverController.getRawAxis(2)), OIConstants.kDriveDeadband),
        true, true),
    m_robotDrive));
    
    m_ODrive.whileTrue(new RunCommand(
        () -> m_robotDrive.drive(
        -MathUtil.applyDeadband((DriveConstants.kODriveSpeed * m_opperator.getRawAxis(1)), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband((DriveConstants.kODriveSpeed * m_opperator.getRawAxis(0)), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(DriveConstants.kODriveSpeed * (m_opperator.getRawAxis(2)), OIConstants.kDriveDeadband),
        false, true),
    m_robotDrive));

    m_OSlow.whileTrue(new RunCommand(
      () -> m_robotDrive.drive(
      -MathUtil.applyDeadband((DriveConstants.kOSlowSpeed * m_opperator.getRawAxis(1)), OIConstants.kDriveDeadband),
      -MathUtil.applyDeadband((DriveConstants.kOSlowSpeed * m_opperator.getRawAxis(0)), OIConstants.kDriveDeadband),
      -MathUtil.applyDeadband(DriveConstants.kOSlowSpeed * (m_opperator.getRawAxis(2)), OIConstants.kDriveDeadband),
      false, true),
    m_robotDrive));

    m_clawTest.toggleOnTrue(new RunCommand(
      () -> m_claw.in(), 
      m_claw));

    m_LowerArm.setDefaultCommand(
      new RunCommand(() -> m_LowerArm.lowerArmHoldPosition(), m_LowerArm)
    );

  }

 

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    PathPlannerTrajectory path = PathPlanner.loadPath("OneLink", new PathConstraints(4, 3));

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        path,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(path.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }
}
