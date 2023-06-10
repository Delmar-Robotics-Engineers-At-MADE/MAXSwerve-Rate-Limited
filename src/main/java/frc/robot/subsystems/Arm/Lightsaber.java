package frc.robot.subsystems.Arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants.SWORD_CONSTANTS;
import frc.robot.subsystems.Cameras.LimelightSubsystem;



public class Lightsaber extends SubsystemBase {

    private static final double kLightsaberNudgeAmount = 100;
    private static final double kYawToleranceDeg = 3;
    private static final double kLightSaberHomeDeg = 9.5;
    

    private TalonSRX m_wristMotor;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;  
    public boolean m_holding = false;
    private final Timer m_timer = new Timer();
    private boolean m_encoderHomed = false;
    private LimelightSubsystem m_limelight;


    public Lightsaber(LimelightSubsystem limelight) {

        m_limelight = limelight;

        m_wristMotor = new TalonSRX(SWORD_CONSTANTS.CLAW_ID);
        m_wristMotor.configFactoryDefault();
        m_wristMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, SWORD_CONSTANTS.kPIDLoopIdx, SWORD_CONSTANTS.kTimeoutMs);
        m_wristMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, SWORD_CONSTANTS.kPIDPositionIdx, SWORD_CONSTANTS.kTimeoutMs);
        // redundantly set below m_wristMotor.setSensorPhase(true);

        // neutral mode
        m_wristMotor.setNeutralMode(NeutralMode.Brake);

        /* Config the peak and nominal outputs */
        m_wristMotor.configNominalOutputForward(0, SWORD_CONSTANTS.kTimeoutMs);
        m_wristMotor.configNominalOutputReverse(0, SWORD_CONSTANTS.kTimeoutMs);
        m_wristMotor.configPeakOutputForward(0.5, SWORD_CONSTANTS.kTimeoutMs);
        m_wristMotor.configPeakOutputReverse(-0.5, SWORD_CONSTANTS.kTimeoutMs);

        /* Config the Velocity closed loop gains in slot0 */
        m_wristMotor.config_kF(SWORD_CONSTANTS.kPIDLoopIdx, SWORD_CONSTANTS.kGains_Velocity.kF, SWORD_CONSTANTS.kTimeoutMs);
        m_wristMotor.config_kP(SWORD_CONSTANTS.kPIDLoopIdx, SWORD_CONSTANTS.kGains_Velocity.kP, SWORD_CONSTANTS.kTimeoutMs);
        m_wristMotor.config_kI(SWORD_CONSTANTS.kPIDLoopIdx, SWORD_CONSTANTS.kGains_Velocity.kI, SWORD_CONSTANTS.kTimeoutMs);
        m_wristMotor.config_kD(SWORD_CONSTANTS.kPIDLoopIdx, SWORD_CONSTANTS.kGains_Velocity.kD, SWORD_CONSTANTS.kTimeoutMs);

        /* Config the Posiiton closed loop gains in slot1 */
        m_wristMotor.config_kF(SWORD_CONSTANTS.kPIDPositionIdx, SWORD_CONSTANTS.kGains_Position.kF, SWORD_CONSTANTS.kTimeoutMs);
        m_wristMotor.config_kP(SWORD_CONSTANTS.kPIDPositionIdx, SWORD_CONSTANTS.kGains_Position.kP, SWORD_CONSTANTS.kTimeoutMs);
        m_wristMotor.config_kI(SWORD_CONSTANTS.kPIDPositionIdx, SWORD_CONSTANTS.kGains_Position.kI, SWORD_CONSTANTS.kTimeoutMs);
        m_wristMotor.config_kD(SWORD_CONSTANTS.kPIDPositionIdx, SWORD_CONSTANTS.kGains_Position.kD, SWORD_CONSTANTS.kTimeoutMs);
        m_wristMotor.configAllowableClosedloopError(SWORD_CONSTANTS.kPIDPositionIdx, SWORD_CONSTANTS.kPIDPositionTolerance, SWORD_CONSTANTS.kTimeoutMs);      

        m_wristMotor.setSensorPhase(false); // invert encoder to match motor

        m_wristMotor.configContinuousCurrentLimit(10, SWORD_CONSTANTS.kTimeoutMs);

        ShuffleboardTab shuffTab = Shuffleboard.getTab("Lightsaber");
        shuffTab.addDouble("Motor V", () -> m_wristMotor.getSelectedSensorVelocity(0));
        shuffTab.addDouble("Motor Pos", () -> m_wristMotor.getSelectedSensorPosition(0));
        shuffTab.addDouble("Motor Err", () -> m_wristMotor.getClosedLoopError(0));
        shuffTab.addDouble("Motor Volt", () -> m_wristMotor.getMotorOutputVoltage());
        shuffTab.addDouble("Motor Stator I", () -> getSwordStatorCurrent());
        shuffTab.addDouble("Motor Supply I", () -> getSwordSupplyCurrent());
        shuffTab.addBoolean("Holding", () -> m_holding);
        shuffTab.addBoolean("Homed", () -> m_encoderHomed);

    }

    public void runSwordClosedLoop (double velocity) {
        m_holding = false;
        m_wristMotor.selectProfileSlot(0, 0);
        m_wristMotor.set(ControlMode.Velocity, velocity);
    }

    public void runSwordOpenLoop(double speed) {
        m_holding = false;
        m_wristMotor.selectProfileSlot(SWORD_CONSTANTS.kPIDLoopIdx, SWORD_CONSTANTS.kPIDLoopIdx);
        m_wristMotor.set(ControlMode.PercentOutput, speed);
    }

    // Run these from RobotContainer like this:
    //      new JoystickButton(m_driverController, Button.kA.value)
    //          .whileTrue(m_claw.in());

    private double getSwordStatorCurrent () {
        return m_wristMotor.getStatorCurrent();
    }

    private double getSwordSupplyCurrent () {
        return m_wristMotor.getSupplyCurrent();
    }

    public void startStallTimer() {
        m_timer.reset();
        m_timer.start();
    }

    public void prepareToHold() {
        m_wristMotor.selectProfileSlot(SWORD_CONSTANTS.kPIDPositionIdx, 0);
        m_wristMotor.setSelectedSensorPosition(0.0);  // reset encoder to 0
        m_holding = false;
    }

    public void hold(double position) {
        m_wristMotor.selectProfileSlot(SWORD_CONSTANTS.kPIDPositionIdx, 0);
        m_wristMotor.set(ControlMode.Position, position);
        m_holding = true;
    }

    public void setEncoderHomed() {
        m_encoderHomed = true;
        m_wristMotor.setSelectedSensorPosition(0.0);  // reset encoder to 0
    }

    public void checkEncoderHomed() {
        double target = m_limelight.getBestLimelightYaw();
        if (Math.abs(target - kLightSaberHomeDeg) < kYawToleranceDeg) {
            setEncoderHomed();
        }
    }

    public void nudgeClosedLoop (boolean clockwise) {
        double nudgeAmount = kLightsaberNudgeAmount;
        if (!clockwise) { 
            nudgeAmount = -nudgeAmount;
        }
        m_wristMotor.selectProfileSlot(SWORD_CONSTANTS.kPIDPositionIdx, 0);
        double currentPosition = m_wristMotor.getSelectedSensorPosition(0);
        System.out.println("setting target enc position to " + currentPosition + " + " + nudgeAmount);
        hold(currentPosition + nudgeAmount);
        // m_wristMotor.set(ControlMode.Position, currentPosition + nudgeAmount);
    }

    public boolean atClosedLoopTarget() {
        // return Math.abs(m_wristMotor.getClosedLoopError()) < 100;
        // return Math.abs(m_sword.getClosedLoopSensorValue() - m_target) < SWORD_CONSTANTS.kPIDPositionTolerance;
        // m_sword.getSelectedSensorVelocity() < 
        return (Math.abs(m_wristMotor.getMotorOutputVoltage()) < 1000)
            && (Math.abs(m_wristMotor.getSelectedSensorPosition(0) - m_wristMotor.getClosedLoopTarget(0)) 
                < SWORD_CONSTANTS.kPIDPositionTolerance * 3);
    }

    // public double getClosedLoopSensorValue() {
    //     return m_wristMotor.getSelectedSensorPosition(0);
    // }
 
}
