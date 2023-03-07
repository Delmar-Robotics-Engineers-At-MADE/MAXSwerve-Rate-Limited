// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Commands.Arm.MoveUpperArmCommand;
import frc.robot.Commands.DriveCommands.Balance;
import frc.robot.Commands.DriveCommands.DriveToAprilTag;
import frc.robot.Commands.DriveCommands.TurnToAprilTagProfiled;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.UpperArmConstants;
import frc.robot.Constants.ControllerConstants.DriverConstants;
import frc.robot.Constants.ControllerConstants.OpperatorConstants;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.subsystems.Arm.LowerArm;
import frc.robot.subsystems.Arm.UpperArmSubsystem;
import frc.robot.subsystems.Cameras.AprilTagSubsystem;
import frc.robot.subsystems.Cameras.LimelightSubsystem;
import frc.robot.subsystems.Swerve.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import frc.robot.Commands.Arm.MoveUpperArmCommand;
import frc.robot.Commands.Arm.HomeUpperArmCommand;
import frc.robot.Commands.Arm.MoveUpperArmCommand;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
// The robot's subsystems
  public final static DriveSubsystem m_robotDrive = new DriveSubsystem();
  private static final Claw m_claw = new Claw();
  private static final UpperArmSubsystem m_upperArm = new UpperArmSubsystem();
  private static final LowerArm m_lowerArm = new LowerArm();
  private static final Balance m_balance = new Balance();
  private static final LimelightSubsystem m_limelight = new LimelightSubsystem();
  private static final AprilTagSubsystem m_aprilTags = new AprilTagSubsystem();


  

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

  // Autonomous options
  private static final String kDock = "Dock";
  private static final String kRightSideDock = "RightSideDock";
  private static final String kLeftSideDock = "LeftSideDock";
  private static final String kSimple = "Simple";

  private ShuffleboardTab m_comp;
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

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
        .toggleOnTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    
    new JoystickButton(m_driverController, DriverConstants.ZERO_HEADING)
        .whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
    
    new JoystickButton(m_driverController, DriverConstants.kAutoBalance)
        .toggleOnTrue(m_balance);

    new JoystickButton(m_opperator, OpperatorConstants.kNudgeUp)
    .toggleOnTrue(new InstantCommand(
        () -> m_upperArm.nudgeClosedLoopByFalconEnc(true), m_upperArm ));

    new JoystickButton(m_opperator, OpperatorConstants.kNudgeDown)
    .toggleOnTrue(new InstantCommand(
        () -> m_upperArm.nudgeClosedLoopByFalconEnc(false), m_upperArm ));
    
    // new JoystickButton(m_opperator, OpperatorConstants.kMoveUpperArmToTarget)
    // .toggleOnTrue(new MoveUpperArmCommand(200, m_upperArm));

    new JoystickButton(m_driverController, DriverConstants.kTurnToTag)
    .toggleOnTrue(new TurnToAprilTagProfiled(0, m_aprilTags, m_robotDrive));

    new JoystickButton(m_driverController, DriverConstants.kDriveToTag)
    .toggleOnTrue(new DriveToAprilTag(0.5, m_aprilTags, m_robotDrive));

    new JoystickButton(m_opperator, OpperatorConstants.kFloorMode)
    .whileTrue(m_lowerArm.lowerArmFloorPosition());
    new JoystickButton(m_opperator, OpperatorConstants.kFloorMode)
    .whileTrue(new HomeUpperArmCommand(m_upperArm));

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
    new JoystickButton(m_opperator, OpperatorConstants.kShootCubeHigh)
    .whileTrue(new HomeUpperArmCommand(m_upperArm));

    new JoystickButton(m_opperator, DriverConstants.kSingleSubstation)
    .whileTrue(m_lowerArm.lowerArmSSsPosition());

    new JoystickButton(m_driverController, OpperatorConstants.kHomeArms)
    .whileTrue(m_lowerArm.homeLowerArm());
    new JoystickButton(m_driverController, OpperatorConstants.kHomeArms)
    .whileTrue(new HomeUpperArmCommand(m_upperArm));

    new JoystickButton(m_driverController, DriverConstants.kLowerArmUp)
    .whileTrue(m_lowerArm.runLowerArmUp());

    new JoystickButton(m_driverController, DriverConstants.kLowerArmDown)
    .whileTrue(m_lowerArm.runLowerArmDown());
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_comp = Shuffleboard.getTab("Competition");

    // config sendable chooser
    m_chooser.setDefaultOption("Default Auto", kSimple);
    m_chooser.addOption("Dock Centre", kDock);
    m_chooser.addOption("Dock Left", kLeftSideDock);
    m_chooser.addOption("Dock Right", kRightSideDock);
    m_comp.add("Auto", m_chooser);
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

    // Configure default commands

    CommandScheduler.getInstance().setDefaultCommand(m_claw, m_claw.stop());

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

    

    m_autoBalance.toggleOnTrue(new Balance());

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
    // Create config for trajectory
    PathPlannerTrajectory path = PathPlanner.loadPath( m_autoSelected, new PathConstraints(2, 2));

    // This is just an example event map. It would be better to have a constant, global event map
    // in your code that will be used by all path following commands.
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("Balance", new Balance());

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
