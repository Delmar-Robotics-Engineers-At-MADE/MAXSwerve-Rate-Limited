// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Commands.Arm.MoveLowerArmCommand;
import frc.robot.Commands.Arm.SwordFight;
import frc.robot.Commands.Arm.SwordHomeByLimelight;
import frc.robot.Commands.Arm.SwordToPosition;
import frc.robot.Commands.Blinkin.DefaultLighting;
import frc.robot.Commands.Blinkin.SignalLightSaber;
import frc.robot.Constants.LowerArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SWORD_CONSTANTS;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.Arm.Lightsaber;
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
  private static final Lightsaber m_lightsaber = new Lightsaber(m_limelight);
  
  // The driver's controller
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);

  //The opperator's controller
  XboxController m_operController = new XboxController(OIConstants.kOpperatorControllerPort);

  private final SequentialCommandGroup m_armToStowPosition = new SequentialCommandGroup(
    new MoveLowerArmCommand(LowerArmConstants.kHomePosition, m_lowerArm)
  );

  private final ParallelCommandGroup m_armToEnGarde = new ParallelCommandGroup(
    new MoveLowerArmCommand(LowerArmConstants.kMidPosition, m_lowerArm)
  );

  // sequences for summer demo

  private final SequentialCommandGroup m_waxOn = new SequentialCommandGroup(
    // upper and lower arm to stow
    new MoveLowerArmCommand(LowerArmConstants.kHomePosition, m_lowerArm),
    new SwordToPosition(SWORD_CONSTANTS.kWaxMove, m_lightsaber),
    new SwordToPosition(0, m_lightsaber),
    new InstantCommand(() -> m_lightsaber.hold(0))
  );

  private final SequentialCommandGroup m_waxOff = new SequentialCommandGroup(
    // upper and lower arm to stow
    new MoveLowerArmCommand(LowerArmConstants.kHomePosition, m_lowerArm),
    new SwordToPosition(-3000, m_lightsaber),
    new SwordToPosition(0, m_lightsaber),
    new InstantCommand(() -> m_lightsaber.hold(0))
  );

  private final SequentialCommandGroup m_homeCommand = new SequentialCommandGroup(
    // upper and lower arm to stow
    new InstantCommand(() -> m_lightsaber.hold(0)),
    new MoveLowerArmCommand(LowerArmConstants.kHomePosition, m_lowerArm),
    new SwordHomeByLimelight(m_limelight, m_lightsaber),
    new InstantCommand(() -> m_lightsaber.hold(0))
  );

  private final SequentialCommandGroup m_swordFight = new SequentialCommandGroup(
    new InstantCommand(() -> m_lightsaber.hold(0)),
    new MoveLowerArmCommand(LowerArmConstants.kMidPosition, m_lowerArm),
    new SwordFight(m_lightsaber, m_limelight, m_blinkin)
  );

  CommandBase m_testCommand = m_waxOn;
  // CommandBase m_testCommand = new RunCommand(() -> m_lightsaber.runClawOpenLoop(1), m_lightsaber);
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

    /****************************** Operator *************************** */

    new JoystickButton(m_operController, Button.kA.value)
      .onTrue(m_homeCommand)
      .onFalse(new InstantCommand(() -> m_homeCommand.cancel()));

      new JoystickButton(m_operController, Button.kY.value)
      .onTrue(m_swordFight)
      .onFalse(new InstantCommand(() -> m_swordFight.cancel()));

      new JoystickButton(m_operController, Button.kB.value)
      .onTrue(m_waxOn)
      .onFalse(new InstantCommand(() -> m_waxOn.cancel()));

      new JoystickButton(m_operController, Button.kX.value)
      .onTrue(m_waxOff)
      .onFalse(new InstantCommand(() -> m_waxOff.cancel()));

    new JoystickButton(m_operController, Button.kLeftBumper.value)
      .toggleOnTrue(new InstantCommand(
          () -> m_lightsaber.nudgeClosedLoop(false), m_lightsaber ));

    new JoystickButton(m_operController, Button.kRightBumper.value)
      .toggleOnTrue(new InstantCommand(
          () -> m_lightsaber.nudgeClosedLoop(true), m_lightsaber ));

  }

  public NoDriveRobotContainer() {
    configureButtonBindings();

    m_lowerArm.setDefaultCommand(
      new RunCommand(
        () -> m_lowerArm.runlowerArmOpenLoop(MathUtil.applyDeadband(
          m_operController.getRightTriggerAxis() - m_operController.getLeftTriggerAxis(),
          OIConstants.kDriveDeadband)), m_lowerArm));
    
    //  m_blinkin.setDefaultCommand(new SignalLightSaber(m_limelight, m_blinkin));

  }


  public Command getAutonomousCommand() {

    // return something bogus
    return null;//new HomeUpperArmCommand(m_upperArm);
  }

  public void zeroHeading() {
    // m_robotDrive.zeroHeading();
  }

}
