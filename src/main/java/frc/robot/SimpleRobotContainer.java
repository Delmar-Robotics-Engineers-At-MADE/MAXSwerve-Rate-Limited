// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Commands.Arm.HoldClawGrip;
import frc.robot.Commands.Arm.MoveClawUntilStall;
import frc.robot.Commands.Arm.PrepareToHold;
import frc.robot.Commands.DriveCommands.MoveToGamepieceProfiled;
import frc.robot.Commands.DriveCommands.TurnToGamepieceProfiled;
import frc.robot.Commands.DriveCommands.UpdateBestGamepieceCommand;
import frc.robot.Constants.CLAW_CONSTANTS;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.subsystems.Arm.UpperArmSubsystem;
import frc.robot.subsystems.Cameras.AprilTagSubsystem;
import frc.robot.subsystems.Cameras.LimelightSubsystem;
import frc.robot.subsystems.Swerve.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class SimpleRobotContainer {

  // The robot's subsystems
  public final static DriveSubsystem m_robotDrive = new DriveSubsystem();
  private static final Claw m_claw = new Claw();
  private static final UpperArmSubsystem m_upperArm = new UpperArmSubsystem();
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
  
  private void configureButtonBindings() {

  // new JoystickButton(m_operController, Button.kA.value)
  //   .whileTrue(new RepeatCommand(new UpdateBestGamepieceCommand(m_limelight)));

  new JoystickButton(m_operController, Button.kA.value)
    .toggleOnTrue(m_moveAndHoldCommand)
    .whileTrue(new RepeatCommand(new TurnToGamepieceProfiled(m_limelight, m_robotDrive)));

  new JoystickButton(m_operController, Button.kY.value)
    .toggleOnTrue(m_moveAndHoldCommand)
    .whileTrue(new RepeatCommand(new MoveToGamepieceProfiled(
      0.3 * DriveConstants.kCrawlSpeed, m_limelight, m_robotDrive)));
    // .whileTrue(new RunCommand(
    //   () -> m_robotDrive.drive(0.3 * DriveConstants.kCrawlSpeed, 0, 0, false, true),
    //   m_robotDrive));

  new JoystickButton(m_driverController, 1)
    .whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

  new JoystickButton(m_operController, Button.kX.value)
    .whileTrue(new RunCommand(() -> m_claw.hold(0), m_claw));

  }

  public SimpleRobotContainer() {
    configureButtonBindings();

    CommandScheduler.getInstance().setDefaultCommand(m_claw, m_claw.stop());

    m_robotDrive.setDefaultCommand(
      new RunCommand(
        () -> m_robotDrive.drive(
        -MathUtil.applyDeadband((DriveConstants.kCrawlSpeed * m_driverController.getRawAxis(1)), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband((DriveConstants.kCrawlSpeed * m_driverController.getRawAxis(0)), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(DriveConstants.kCrawlSpeed * (m_driverController.getRawAxis(2)), OIConstants.kDriveDeadband),
        true, true),
        m_robotDrive));
  }


  public Command getAutonomousCommand() {

    // return something bogus
    return null;//new HomeUpperArmCommand(m_upperArm);
  }

  public void zeroHeading() {
    m_robotDrive.zeroHeading();
  }

}
