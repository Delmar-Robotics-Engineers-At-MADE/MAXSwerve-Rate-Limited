package frc.robot.subsystems.Arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.UpperArmConstants;


public class UpperArmSubsystem extends SubsystemBase {

    private final WPI_TalonFX m_upperArmMotor = new WPI_TalonFX(UpperArmConstants.UPPER_ARM_MOTOR_ID);
    private final AnalogPotentiometer m_potmeter = new AnalogPotentiometer(0);
    private final Encoder m_encoder = new Encoder(0, 1, 
                        false, Encoder.EncodingType.k4X);
    public boolean m_encoderHomed = false;
    private ShuffleboardTab m_driveBaseTab;

    // Constructor
    public UpperArmSubsystem() {
        m_upperArmMotor.setNeutralMode(NeutralMode.Brake);

        // planning to use open-loop control on Falcon, plus our own PID controller using Rev encoder, so don't need PID settings

        // m_upperArmMotor.Config_kF(0, kFtuned, 30); // 2022 shooter Falcons were 30
        // m_upperArmMotor.Config_kP(0, kPtuned, 30);
        // m_upperArmMotor.Config_kI(0, 0.0, 30); // was .00005 in 2020
        // m_upperArmMotor.Config_kD(0, kDtuned, 30); // was 0 in 2020
        // m_upperArmMotor.ConfigClosedloopRamp(kRampTuned);
        // m_upperArmMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 30);

        m_driveBaseTab = Shuffleboard.getTab("Arm");
        m_driveBaseTab.addDouble("Shoulder Pot", () -> m_potmeter.get());
        m_driveBaseTab.add("Shoulder Enc", m_encoder);
        m_driveBaseTab.addBoolean("Homed", () -> m_encoderHomed);
    }

    public double encoderPosition() {
        return m_encoder.getDistance();
    }

    public double potPosition() {
        return m_potmeter.get();
    }

    public void moveOpenLoop (double powerPercent) {
        m_upperArmMotor.set(ControlMode.PercentOutput, powerPercent);
        System.out.println("Falcon 0 is at" + powerPercent);
    }

    public void setEncoderHomed() {
        m_encoderHomed = true;
    }

}