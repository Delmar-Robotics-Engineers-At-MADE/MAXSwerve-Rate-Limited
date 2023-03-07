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

    /**
     * 
     * @param position target lower arm position
     */
    private void runLowerArmClosedLoop(double position) {
        m_elbowPIDController.setReference(position, CANSparkMax.ControlType.kDutyCycle);
        System.out.println("lower arm:" + getElbowPos() +" " + position);
    
    }

    public CommandBase move() {
        return this.run(() -> runLowerArmClosedLoop(m_position));
    }

    public CommandBase homeLowerArm() {
        return this.run(() -> runLowerArmClosedLoop(LowerArmConstants.kHomePosition));
    }

    public CommandBase lowerArmFloorPosition() {
        return this.run(() -> runLowerArmClosedLoop(LowerArmConstants.kFloorPosition));
    }

    public CommandBase lowerArmHighPosition() {
        return this.run(() -> runLowerArmClosedLoop(LowerArmConstants.kHighPosition));
    }

    public CommandBase lowerArmShootPosition() {
        return this.run(() -> runLowerArmClosedLoop(LowerArmConstants.kShootPosition));
    }

    public CommandBase lowerArmMidPosition() {
        return this.run(() -> runLowerArmClosedLoop(LowerArmConstants.kMidPosition));
    }

    public CommandBase lowerArmSSsPosition() {
        return this.run(() -> runLowerArmClosedLoop(LowerArmConstants.kSSsPosition));
    }

    public CommandBase lowerArmHoldPosition() {
        return this.run(() -> runLowerArmClosedLoop(m_elbowEncoder.getPosition()));
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


