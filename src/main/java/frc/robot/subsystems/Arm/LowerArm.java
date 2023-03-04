package frc.robot.subsystems.Arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LowerArmConstants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;

public class LowerArm extends SubsystemBase {
    private final CANSparkMax m_elbow;
    private final AbsoluteEncoder m_elbowEncoder;
    private SparkMaxPIDController m_elbowPIDController;
    private double m_position;
    public ShuffleboardTab m_dashboardTab;

    public GenericEntry m_positionSettingEntry;
    public boolean m_lowerArmHomed;
    

    public LowerArm() {
        m_elbow = new CANSparkMax(LowerArmConstants.LOWER_ARM_MOTOR_ID, MotorType.kBrushless);
        m_elbowEncoder = m_elbow.getAbsoluteEncoder(Type.kDutyCycle);
        m_elbow.restoreFactoryDefaults();
        m_elbow.setSmartCurrentLimit(20);
        m_elbow.setIdleMode(IdleMode.kBrake);
        m_elbowEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
        m_elbowPIDController = m_elbow.getPIDController();
        m_elbowPIDController.setSmartMotionMinOutputVelocity(LowerArmConstants.minVelocity, 0);
        m_elbowPIDController.setSmartMotionMaxAccel(LowerArmConstants.maxAccel, 0);
        m_elbowPIDController.setSmartMotionMaxVelocity(LowerArmConstants.maxVelocity, 0);
        m_elbowPIDController.setSmartMotionAllowedClosedLoopError(LowerArmConstants.allowedErr, 0);
        
        m_elbowPIDController.setP(LowerArmConstants.kP);
        m_elbowPIDController.setI(LowerArmConstants.kI);
        m_elbowPIDController.setD(LowerArmConstants.kD);
        m_elbowPIDController.setFF(LowerArmConstants.kFF);
        m_elbowPIDController.setOutputRange(LowerArmConstants.kMinOutput, LowerArmConstants.kMaxOutput);
        m_elbow.burnFlash();

        m_lowerArmHomed = false;

        m_dashboardTab = Shuffleboard.getTab("Arm");
        m_positionSettingEntry = m_dashboardTab.add("Set Lower Arm", m_position).getEntry();
        m_positionSettingEntry.getDouble(m_position);
        m_dashboardTab.addBoolean("LowerArm Homed", () -> m_lowerArmHomed);
    // m_defaultSettingEntry = m_dashboardTab.add("Default", m_defaultSetting).getEntry();
    // m_conesSettingEntry = m_dashboardTab.add("Cubes", m_signalCubes).getEntry();
    }

    public CommandBase move() {
        return this.runOnce(() -> m_elbowPIDController.setReference(m_position, CANSparkMax.ControlType.kDutyCycle));
    }

    public CommandBase homeLowerArm() {
        return this.runOnce(() -> m_elbowPIDController.setReference(LowerArmConstants.kHomePosition, CANSparkMax.ControlType.kDutyCycle));
    }

    public CommandBase lowerArmFloorPosition() {
        return this.runOnce(() -> m_elbowPIDController.setReference(LowerArmConstants.kFloorPosition, CANSparkMax.ControlType.kDutyCycle));
    }

    public CommandBase lowerArmHighPosition() {
        return this.runOnce(() -> m_elbowPIDController.setReference(LowerArmConstants.kHighPosition, CANSparkMax.ControlType.kDutyCycle));
    }

    public CommandBase lowerArmShootPosition() {
        return this.runOnce(() -> m_elbowPIDController.setReference(LowerArmConstants.kShootPosition, CANSparkMax.ControlType.kDutyCycle));
    }

    public CommandBase lowerArmMidPosition() {
        return this.runOnce(() -> m_elbowPIDController.setReference(LowerArmConstants.kMidPosition, CANSparkMax.ControlType.kDutyCycle));
    }

    public CommandBase lowerArmSSsPosition() {
        return this.runOnce(() -> m_elbowPIDController.setReference(LowerArmConstants.kSSsPosition, CANSparkMax.ControlType.kDutyCycle));
    }

    public CommandBase lowerArmHoldPosition() {
        return this.runOnce(() -> m_elbowPIDController.setReference(m_elbowEncoder.getPosition(), CANSparkMax.ControlType.kDutyCycle));
    }

    public CommandBase runLowerArmUp() {
        return this.run(() -> m_elbow.set(LowerArmConstants.kManualSpeed));
    }

    public CommandBase runLowerArmDown() {
        return this.run(() -> m_elbow.set(-1 * LowerArmConstants.kManualSpeed));
    }

    public double getElbowPos() {
        return m_elbowEncoder.getPosition();
    }

    public boolean setElbowHomed(){
        return m_lowerArmHomed = true;
    }

}


