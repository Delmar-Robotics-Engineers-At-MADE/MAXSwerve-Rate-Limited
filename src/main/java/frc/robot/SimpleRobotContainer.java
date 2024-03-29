// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Commands.Arm.HoldClawGrip;
import frc.robot.Commands.Arm.HoldLowerArmCommand;
import frc.robot.Commands.Arm.MoveClawUntilStall;
import frc.robot.Commands.Arm.MoveLowerArmCommand;
import frc.robot.Commands.Arm.MoveUpperArmCommand;
import frc.robot.Commands.Arm.PrepareToHold;
import frc.robot.Commands.Arm.ShootCubeTimed;
import frc.robot.Commands.Blinkin.DefaultLighting;
import frc.robot.Commands.DriveCommands.DriveToAprilTag;
import frc.robot.Commands.DriveCommands.MoveToGamepieceProfiled;
import frc.robot.Commands.DriveCommands.TurnToGamepieceProfiled;
import frc.robot.Commands.DriveCommands.UpdateBestAprilTag;
import frc.robot.Commands.DriveCommands.UpdateBestGamepieceCommand;
import frc.robot.Commands.DriveCommands.SearchForAprilTagProfiled;
import frc.robot.Commands.DriveCommands.StrafeToAprilTagProfiled;
import frc.robot.Commands.DriveCommands.TurnToAprilTagProfiled;
import frc.robot.Constants.CLAW_CONSTANTS;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LowerArmConstants;
import frc.robot.Constants.UpperArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.subsystems.Arm.LowerArm;
import frc.robot.subsystems.Arm.UpperArmSubsystem;
import frc.robot.subsystems.Cameras.AprilTagSubsystem;
import frc.robot.subsystems.Cameras.LimelightSubsystem;
import frc.robot.subsystems.Swerve.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class SimpleRobotContainer {

  // The robot's subsystems
  public final static DriveSubsystem m_robotDrive = new DriveSubsystem();
  private static final Claw m_claw = new Claw();
  private static final UpperArmSubsystem m_upperArm = new UpperArmSubsystem();
  private static final LowerArm m_lowerArm = new LowerArm();
  private static final LimelightSubsystem m_limelight = new LimelightSubsystem();
  private static final AprilTagSubsystem m_aprilTags = new AprilTagSubsystem();
  private static final BlinkinSubsystem m_blinkin = new BlinkinSubsystem();
  
  // The driver's controller
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);

  //The opperator's controller
  XboxController m_operController = new XboxController(OIConstants.kOpperatorControllerPort);

  // sequence for running claw to stall and then holding:
  private final SequentialCommandGroup m_moveAndHoldCommand = new SequentialCommandGroup(
    new InstantCommand(() -> m_claw.startStallTimer()),
    new MoveClawUntilStall(CLAW_CONSTANTS.kInVelocity, m_claw), 
    new PrepareToHold(m_claw),
    new HoldClawGrip(0.0, m_claw)
  );

  private final SequentialCommandGroup m_driveToAprilTagCommand = new SequentialCommandGroup(
    new TurnToAprilTagProfiled(0, m_aprilTags, m_robotDrive),
    new StrafeToAprilTagProfiled(m_aprilTags, m_robotDrive)
  );

  private final SequentialCommandGroup m_armToStowPosition = new SequentialCommandGroup(
    new MoveLowerArmCommand(LowerArmConstants.kFloorPosition, m_lowerArm),
    new MoveUpperArmCommand(UpperArmConstants.kSummerIntakePosition, m_upperArm),
    new MoveLowerArmCommand(LowerArmConstants.kHomePosition, m_lowerArm)
  );

  private final ParallelCommandGroup m_armToReturnPosition = new ParallelCommandGroup(
    new MoveUpperArmCommand(UpperArmConstants.kSummerReturnPosition, m_upperArm),
    new MoveLowerArmCommand(LowerArmConstants.kFloorPosition, m_lowerArm)
  );

  // sequences for summer demo

  private final SequentialCommandGroup m_summerReturnOnly = new SequentialCommandGroup(
    // return cube
    new ShootCubeTimed(1, m_claw),
    // upper and lower arm to stow
    new MoveUpperArmCommand(UpperArmConstants.kSummerIntakePosition, m_upperArm),
    new MoveLowerArmCommand(LowerArmConstants.kHomePosition, m_lowerArm)
  );

  private final SequentialCommandGroup m_summerCollectAndReturn = new SequentialCommandGroup(
    // upper and lower arm to intake position
    new MoveLowerArmCommand(LowerArmConstants.kFloorPosition, m_lowerArm),
    new MoveUpperArmCommand(UpperArmConstants.kSummerIntakePosition, m_upperArm),
    // drive forward and rotate toward game piece until collected
    new MoveToGamepieceProfiled(
      0.3 * DriveConstants.kCrawlSpeed, m_limelight, m_robotDrive, m_claw),
    new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false, false)),
    // raise lower arm before turning
    new MoveLowerArmCommand(LowerArmConstants.kHomePosition, m_lowerArm),
    // back up if too close to wall
    // new DriveToAprilTag(CameraConstants.kSummerAprilTagDistanceBackup, m_aprilTags, m_robotDrive),
    // search for nearest April tag
    new SearchForAprilTagProfiled(m_aprilTags, m_robotDrive),
    new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false, false)),
    // upper and lower arm to return position
    new MoveLowerArmCommand(LowerArmConstants.kFloorPosition, m_lowerArm),
    new MoveUpperArmCommand(UpperArmConstants.kSummerReturnPosition, m_upperArm),
    // drive to April Tag
    new DriveToAprilTag(CameraConstants.kSummerAprilTagDistance, m_aprilTags, m_robotDrive),
    new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false, false)),
    // return cube
    // new RunCommand(() -> m_claw.runClawClosedLoop(CLAW_CONSTANTS.kCubeOutVelocity))
    new ShootCubeTimed(1, m_claw),
    // upper and lower arm to stow
    new MoveUpperArmCommand(UpperArmConstants.kSummerIntakePosition, m_upperArm),
    new MoveLowerArmCommand(LowerArmConstants.kHomePosition, m_lowerArm)
  );

  private final SequentialCommandGroup m_summerTurnAndReturn = new SequentialCommandGroup(
    // upper and lower arm to intake position
    new MoveLowerArmCommand(LowerArmConstants.kFloorPosition, m_lowerArm),
    new MoveUpperArmCommand(UpperArmConstants.kSummerIntakePosition, m_upperArm),
    // rotate toward game piece until collected
    new TurnToGamepieceProfiled(m_limelight, m_robotDrive, m_claw),
    // upper arm to return position
    new MoveUpperArmCommand(UpperArmConstants.kSummerReturnPosition, m_upperArm),
    // return cube
    new ShootCubeTimed(1, m_claw),
    // upper and lower arm to stow
    new MoveUpperArmCommand(UpperArmConstants.kSummerIntakePosition, m_upperArm),
    new MoveLowerArmCommand(LowerArmConstants.kHomePosition, m_lowerArm)
  );

  CommandBase m_testCommand = new RunCommand(() -> m_claw.runClawOpenLoop(1), m_claw);
  // CommandBase m_testCommand = m_summerReturnOnly;
  // CommandBase m_testCommand = new SearchForAprilTagProfiled(m_aprilTags, m_robotDrive);
  // CommandBase m_testCommand = m_armToIntakePosition;
  // CommandBase m_testCommand = m_lowerArm.lowerArmFloorPosition();
  // CommandBase m_testCommand = m_driveToAprilTagCommand ; // m_lowerArm.lowerArmMidPosition();
  // CommandBase m_testCommand = new RepeatCommand(new StrafeToAprilTagProfiled(m_aprilTags, m_robotDrive));
  // CommandBase m_testCommand = new RepeatCommand(new TurnToAprilTagProfiled(0, m_aprilTags, m_robotDrive));
  
  private void configureButtonBindings() {

    /****************************** Driver *************************** */

    // new JoystickButton(m_driverController, 2)
    //   .whileTrue(new RepeatCommand(new UpdateBestAprilTag(m_aprilTags)));

    new JoystickButton(m_driverController, 2)
      .onTrue(m_testCommand)
      .onFalse(new InstantCommand(() -> m_testCommand.cancel()));

    new JoystickButton(m_driverController, 1)
      .whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    /****************************** Operator *************************** */

    new JoystickButton(m_operController, Button.kA.value)
      .onTrue(m_moveAndHoldCommand)
      .whileTrue(m_summerTurnAndReturn)
      .onFalse(new InstantCommand(() -> m_moveAndHoldCommand.cancel()))
      .onFalse(new RunCommand(() -> m_claw.runClawOpenLoop(0), m_claw));

    new JoystickButton(m_operController, Button.kY.value)
      .onTrue(m_moveAndHoldCommand)
      .whileTrue(m_summerCollectAndReturn)
      .onFalse(new InstantCommand(() -> m_moveAndHoldCommand.cancel()));

    new JoystickButton(m_operController, Button.kB.value)
      .toggleOnTrue(m_armToStowPosition);
      //.toggleOnTrue(new MoveUpperArmCommand(UpperArmConstants.kSummerIntakePosition, m_upperArm));

    new JoystickButton(m_operController, Button.kX.value)
      //.toggleOnTrue(new MoveUpperArmCommand(UpperArmConstants.kSummerReturnPosition, m_upperArm));
      .toggleOnTrue(m_armToReturnPosition);

    new JoystickButton(m_operController, Button.kLeftBumper.value)
      .toggleOnTrue(new InstantCommand(
          () -> m_upperArm.nudgeClosedLoopByFalconEnc(true), m_upperArm ));

    new JoystickButton(m_operController, Button.kRightBumper.value)
      .toggleOnTrue(new InstantCommand(
          () -> m_upperArm.nudgeClosedLoopByFalconEnc(false), m_upperArm ));
    

  }

  public SimpleRobotContainer() {
    configureButtonBindings();

    // CommandScheduler.getInstance().setDefaultCommand(m_claw, m_claw.stop());

    m_robotDrive.setDefaultCommand(
      new RunCommand(
        () -> m_robotDrive.drive(
        -MathUtil.applyDeadband((DriveConstants.kCrawlSpeed * m_driverController.getRawAxis(1)), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband((DriveConstants.kCrawlSpeed * m_driverController.getRawAxis(0)), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(DriveConstants.kCrawlSpeed * (m_driverController.getRawAxis(2)), OIConstants.kDriveDeadband),
        true, true),
        m_robotDrive));

    m_lowerArm.setDefaultCommand(
      new RunCommand(
        () -> m_lowerArm.runlowerArmOpenLoop(MathUtil.applyDeadband(
          m_operController.getRightTriggerAxis() - m_operController.getLeftTriggerAxis(),
          OIConstants.kDriveDeadband)), m_lowerArm));
    
     m_blinkin.setDefaultCommand(new DefaultLighting(m_blinkin));

  }


  public Command getAutonomousCommand() {

    // return something bogus
    return null;//new HomeUpperArmCommand(m_upperArm);
  }

  public void zeroHeading() {
    m_robotDrive.zeroHeading();
  }

}
