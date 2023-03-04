package frc.robot.subsystems.Arm;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.UpperArmConstants;


public class UpperArmSubsystem extends SubsystemBase {

    private final WPI_TalonFX m_upperArmMotor = new WPI_TalonFX(UpperArmConstants.UPPER_ARM_MOTOR_ID);
    private final AnalogPotentiometer m_potmeter = new AnalogPotentiometer(0);
    private final Encoder m_encoder = new Encoder(0, 1, 
                        true, Encoder.EncodingType.k4X);
    public boolean m_encoderHomed = false;
    private ShuffleboardTab m_armTab;
    public GenericEntry m_nudgeDashboardEntry;
    private GenericEntry m_falconPEntry;

    // Constructor
    public UpperArmSubsystem() {
        m_upperArmMotor.setNeutralMode(NeutralMode.Brake);

        // planning to use open-loop control on Falcon, plus our own PID controller using Rev encoder, so don't need PID settings

        ErrorCode err = m_upperArmMotor.configFactoryDefault(); if (err.value != 0) {System.out.println("Falcon config err: " + err.value);}
        err = m_upperArmMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30); if (err.value != 0) {System.out.println("Falcon config err: " + err.value);}
        err = m_upperArmMotor.setSelectedSensorPosition(0.0) ; if (err.value != 0) {System.out.println("Falcon config err: " + err.value);}

        // note whether already homed
        checkEncoderHomed();

        m_armTab = Shuffleboard.getTab("Arm");
        m_armTab.addDouble("Shoulder Pot", () -> m_potmeter.get());
        m_armTab.add("Shoulder Enc", m_encoder);
        m_armTab.addBoolean("Homed", () -> m_encoderHomed);
        m_armTab.addDouble("Falcon Enc", () -> m_upperArmMotor.getSelectedSensorPosition());
        m_armTab.addDouble("Falcon Error", () -> m_upperArmMotor.getClosedLoopError());
        m_nudgeDashboardEntry = m_armTab.add("Nudge Amount", Constants.UpperArmConstants.kFalconTestNudgeAmount).getEntry();
        m_falconPEntry = m_armTab.add("Falcon P", Constants.UpperArmConstants.kFalconP).getEntry();
    }

    public double encoderPosition() {
        return m_encoder.getDistance();
    }

    public double potPosition() {
        return m_potmeter.get();
    }

    private double clamp (double x, double limitLow, double limitHigh) {
        double result = Math.max(x, limitLow);
        result = Math.min(result, limitHigh);
        return result;
    }

    public void moveOpenLoop (double powerPercent) {
        powerPercent = clamp (powerPercent, -Constants.UpperArmConstants.kMaxFalconPower, 
                                             Constants.UpperArmConstants.kMaxFalconPower);
        m_upperArmMotor.set(ControlMode.PercentOutput, powerPercent);
        System.out.println("Falcon power: " + powerPercent);
    }

    public void setEncoderHomed() {
        m_encoderHomed = true;
        m_encoder.reset();
    }

    private void checkEncoderHomed() {
        if ( Math.abs(potPosition() - Constants.UpperArmConstants.kHomePotmeterValue) 
                < Constants.UpperArmConstants.kPotmeterTolerance) {
            setEncoderHomed();
            }
    }

    public void nudgeClosedLoopByFalconEnc (boolean up) {
        double nudgeAmount = m_nudgeDashboardEntry.getDouble(0);
        checkEncoderHomed();
        if (!up) { // going down
            nudgeAmount = -nudgeAmount;
        }
        if (!up && m_encoderHomed && encoderPosition() < 20) {
            // don't go down below home
            System.out.println("don't go down anymore !!!!!!!!!!!!");
            // but allow it, was... m_upperArmMotor.set(ControlMode.PercentOutput, 0.0);
        }
        System.out.println("ok to go");
        double kP = m_falconPEntry.getDouble(0);
        ErrorCode err = m_upperArmMotor.config_kP(0, kP, 30); if (err.value != 0) {System.out.println("Falcon config err: " + err.value);}
        double currentPosition = m_upperArmMotor.getSelectedSensorPosition();
        System.out.println("setting target enc position to " + currentPosition + " + " + nudgeAmount);
        m_upperArmMotor.set(ControlMode.Position, currentPosition + nudgeAmount);
    }

    public double getFalconEncPosition () {
        return m_upperArmMotor.getSelectedSensorPosition();
    }

}