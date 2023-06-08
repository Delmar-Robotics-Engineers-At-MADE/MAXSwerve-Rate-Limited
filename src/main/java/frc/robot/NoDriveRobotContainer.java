// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Commands.Arm.MoveLowerArmCommand;
import frc.robot.Commands.Blinkin.DefaultLighting;
import frc.robot.Commands.Blinkin.SignalLightSaber;
import frc.robot.Constants.LowerArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.Arm.LowerArm;
import frc.robot.subsystems.Cameras.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class NoDriveRobotContainer {

  // The robot's subsystems
  private static final LowerArm m_lowerArm = new LowerArm();
  private static final LimelightSubsystem m_limelight = new LimelightSubsystem();
  private static final BlinkinSubsystem m_blinkin = new BlinkinSubsystem();
  
  // The driver's controller
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);

  //The opperator's controller
  XboxController m_operController = new XboxController(OIConstants.kOpperatorControllerPort);

  private final SequentialCommandGroup m_armToStowPosition = new SequentialCommandGroup(
    new MoveLowerArmCommand(LowerArmConstants.kFloorPosition, m_lowerArm),
    new MoveLowerArmCommand(LowerArmConstants.kHomePosition, m_lowerArm)
  );

  private final ParallelCommandGroup m_armToReturnPosition = new ParallelCommandGroup(
    new MoveLowerArmCommand(LowerArmConstants.kFloorPosition, m_lowerArm)
  );

  // sequences for summer demo

  private final SequentialCommandGroup m_summerReturnOnly = new SequentialCommandGroup(
    // upper and lower arm to stow
    new MoveLowerArmCommand(LowerArmConstants.kHomePosition, m_lowerArm)
  );

  private final SequentialCommandGroup m_summerCollectAndReturn = new SequentialCommandGroup(
    // upper and lower arm to intake position
    new MoveLowerArmCommand(LowerArmConstants.kFloorPosition, m_lowerArm),
    // raise lower arm before turning
    new MoveLowerArmCommand(LowerArmConstants.kHomePosition, m_lowerArm),
    // upper and lower arm to return position
    new MoveLowerArmCommand(LowerArmConstants.kFloorPosition, m_lowerArm),
    // upper and lower arm to stow
    new MoveLowerArmCommand(LowerArmConstants.kHomePosition, m_lowerArm)
  );

  private final SequentialCommandGroup m_summerTurnAndReturn = new SequentialCommandGroup(
    // upper and lower arm to intake position
    new MoveLowerArmCommand(LowerArmConstants.kFloorPosition, m_lowerArm),
    // upper and lower arm to stow
    new MoveLowerArmCommand(LowerArmConstants.kHomePosition, m_lowerArm)
  );

  CommandBase m_testCommand = m_summerReturnOnly;
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

    /****************************** Operator *************************** */

    new JoystickButton(m_operController, Button.kA.value)
      .whileTrue(m_summerTurnAndReturn);

    new JoystickButton(m_operController, Button.kY.value)
      .whileTrue(m_summerCollectAndReturn);

    new JoystickButton(m_operController, Button.kB.value)
      .toggleOnTrue(m_armToStowPosition);

    new JoystickButton(m_operController, Button.kX.value)
      .toggleOnTrue(m_armToReturnPosition);

  }

  public NoDriveRobotContainer() {
    configureButtonBindings();

    m_lowerArm.setDefaultCommand(
      new RunCommand(
        () -> m_lowerArm.runlowerArmOpenLoop(MathUtil.applyDeadband(
          m_operController.getRightTriggerAxis() - m_operController.getLeftTriggerAxis(),
          OIConstants.kDriveDeadband)), m_lowerArm));
    
     m_blinkin.setDefaultCommand(new SignalLightSaber(m_limelight, m_blinkin));

  }


  public Command getAutonomousCommand() {

    // return something bogus
    return null;//new HomeUpperArmCommand(m_upperArm);
  }

  public void zeroHeading() {
    // m_robotDrive.zeroHeading();
  }

}
