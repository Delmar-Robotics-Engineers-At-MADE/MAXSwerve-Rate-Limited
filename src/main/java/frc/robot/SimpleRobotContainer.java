// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Commands.DriveCommands.TurnToGamepieceProfiled;
import frc.robot.Commands.DriveCommands.UpdateBestGamepieceCommand;
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

  private void configureButtonBindings() {
    new JoystickButton(m_operController, Button.kA.value)
    .whileTrue(new RepeatCommand(new UpdateBestGamepieceCommand(m_limelight)));

  new JoystickButton(m_operController, Button.kY.value)
    .whileTrue(new RepeatCommand(new TurnToGamepieceProfiled(m_limelight, m_robotDrive)));

  new JoystickButton(m_driverController, 1)
    .whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

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
