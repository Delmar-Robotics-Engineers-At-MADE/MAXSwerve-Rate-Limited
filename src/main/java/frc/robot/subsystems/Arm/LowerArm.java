package frc.robot.subsystems.Arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
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
    private final CANSparkMax m_elbowChain;
    private final AbsoluteEncoder m_elbowEncoder;
    private SparkMaxPIDController m_elbowPIDController;
    private double m_position;
    public ShuffleboardTab m_dashboardTab;

    public GenericEntry m_positionSettingEntry;
    // public boolean m_lowerArmHomed;
    private double m_holdposition;
    

    public LowerArm() {
        m_elbow = new CANSparkMax(LowerArmConstants.LOWER_ARM_MOTOR_ID, MotorType.kBrushless);
        m_elbow.restoreFactoryDefaults();
        m_elbow.setSmartCurrentLimit(60);
        m_elbow.setIdleMode(IdleMode.kBrake);
        m_elbow.setInverted(true);
        m_elbowEncoder = m_elbow.getAbsoluteEncoder(Type.kDutyCycle);
        m_elbowPIDController = m_elbow.getPIDController();
        m_elbowPIDController.setFeedbackDevice(m_elbowEncoder);
        m_elbowEncoder.setPositionConversionFactor(1);
        m_elbowEncoder.setInverted(true);
        m_elbowPIDController.setP(LowerArmConstants.kP, 0);
        m_elbowPIDController.setI(LowerArmConstants.kI, 0);
        m_elbowPIDController.setD(LowerArmConstants.kD, 0);
        m_elbowPIDController.setFF(LowerArmConstants.kFF, 0);
        //m_elbowPIDController.setOutputRange(LowerArmConstants.kMinOutput, LowerArmConstants.kMaxOutput, 0);
        m_elbow.burnFlash();

        m_elbowChain = new CANSparkMax(LowerArmConstants.LOWER_ARM_CHAIN_MOTOR_ID, MotorType.kBrushless);
        m_elbowChain.restoreFactoryDefaults();
        m_elbowChain.setSmartCurrentLimit(60);
        m_elbowChain.setIdleMode(IdleMode.kBrake);
        m_elbowChain.follow(m_elbow, true);

        m_elbowPIDController.setReference(0, CANSparkMax.ControlType.kVelocity, 0);
        m_holdposition = m_elbowEncoder.getPosition();

        // m_lowerArmHomed = false;

        m_dashboardTab = Shuffleboard.getTab("Arm");
        m_positionSettingEntry = m_dashboardTab.add("Set Lower Arm", m_position).getEntry();
        m_positionSettingEntry.getDouble(m_position);
        // m_dashboardTab.addBoolean("LowerArm Homed", () -> m_lowerArmHomed);
        m_dashboardTab.addDouble("elbow pos", () -> getElbowPos());
        m_dashboardTab.addDouble("elbowHoldPosition", () -> m_holdposition);
        
    // m_defaultSettingEntry = m_dashboardTab.add("Default", m_defaultSetting).getEntry();
    // m_conesSettingEntry = m_dashboardTab.add("Cubes", m_signalCubes).getEntry();
    }

    /**
     * 
     * @param position target lower arm position
     */
    public void runLowerArmClosedLoop(double position) {
        m_elbowPIDController.setReference(position, CANSparkMax.ControlType.kPosition, 0);
        System.out.println("Elbow applied output: " + m_elbowChain.getAppliedOutput());
        System.out.println("Elbow output current: " + m_elbowChain.getOutputCurrent());
        // System.out.println("lower arm:" + getElbowPos() +" " + position);
        // checkElbowHomed();
        m_holdposition = m_elbowEncoder.getPosition();
    }

    public void holdLowerArm(){
        // seems to be now way to set tolerance on spark max built-in PID, so do our own
        double pos = m_elbowEncoder.getPosition();
        if (Math.abs(m_holdposition - pos) > LowerArmConstants.kTolerance) {
            m_elbowPIDController.setReference(0.15/*m_holdposition*/, CANSparkMax.ControlType.kPosition, 0);
        } else {
            m_elbow.set(0);
        }
        System.out.println("holding lower arm to " + m_holdposition + ", actual: " + m_elbowEncoder.getPosition());
    }

    public void runlowerArmOpenLoop(double speed) {
        m_elbow.set(speed);
        m_holdposition = m_elbowEncoder.getPosition();
       //System.out.println("Elbow open loop power: " + speed);
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
        //return this.run(() -> m_elbowPIDController.setReference(m_holdposition, ControlType.kDutyCycle));
        return this.run(() -> runLowerArmClosedLoop(m_holdposition));
    }

    public CommandBase runLowerArmUp() {
        return this.run(() -> runlowerArmOpenLoop(-15));
    }

    public CommandBase runLowerArmDown() {
        return this.run(() -> runlowerArmOpenLoop(15));
    }

    public double getElbowPos() {
        return m_elbowEncoder.getPosition();
    }

    // public boolean setElbowHomed(){
    //     return m_lowerArmHomed = true;
    // }

    // private void checkElbowHomed() {
    //     if (getElbowPos() < LowerArmConstants.homeTolerance) {
    //         setElbowHomed();
    //     }
    // }

}


