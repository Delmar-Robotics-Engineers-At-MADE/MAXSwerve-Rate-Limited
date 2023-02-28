package frc.robot.subsystems.Arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CLAW_CONSTANTS;



public class Claw extends SubsystemBase {
    private TalonSRX m_clawMotor;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;  
 

    
    public Claw() {
        m_clawMotor = new TalonSRX(CLAW_CONSTANTS.CLAW_ID);
        m_clawMotor.configFactoryDefault();
        m_clawMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, CLAW_CONSTANTS.kPIDLoopIdx, CLAW_CONSTANTS.kTimeoutMs);
        m_clawMotor.setSensorPhase(true);

        /* Config the peak and nominal outputs */
        m_clawMotor.configNominalOutputForward(0, CLAW_CONSTANTS.kTimeoutMs);
        m_clawMotor.configNominalOutputReverse(0, CLAW_CONSTANTS.kTimeoutMs);
        m_clawMotor.configPeakOutputForward(1, CLAW_CONSTANTS.kTimeoutMs);
        m_clawMotor.configPeakOutputReverse(-1, CLAW_CONSTANTS.kTimeoutMs);

        /* Config the Velocity closed loop gains in slot0 */
        m_clawMotor.config_kF(CLAW_CONSTANTS.kPIDLoopIdx, CLAW_CONSTANTS.kGains_Velocit.kF, CLAW_CONSTANTS.kTimeoutMs);
        m_clawMotor.config_kP(CLAW_CONSTANTS.kPIDLoopIdx, CLAW_CONSTANTS.kGains_Velocit.kP, CLAW_CONSTANTS.kTimeoutMs);
        m_clawMotor.config_kI(CLAW_CONSTANTS.kPIDLoopIdx, CLAW_CONSTANTS.kGains_Velocit.kI, CLAW_CONSTANTS.kTimeoutMs);
        m_clawMotor.config_kD(CLAW_CONSTANTS.kPIDLoopIdx, CLAW_CONSTANTS.kGains_Velocit.kD, CLAW_CONSTANTS.kTimeoutMs);

    }

    public CommandBase in() {
       return this.run(()-> m_clawMotor.set(ControlMode.Velocity, CLAW_CONSTANTS.kInVelocity));
    }
    public CommandBase coneOut() {
        return this.run(()-> m_clawMotor.set(ControlMode.Velocity, CLAW_CONSTANTS.kConeOutVelocity));
    }
    public CommandBase cubeOut() {
        return this.run(()-> m_clawMotor.set(ControlMode.Velocity, CLAW_CONSTANTS.kCubeOutVelocity));
    }
    public CommandBase shoot() {
        return this.run(()-> m_clawMotor.set(ControlMode.Velocity, CLAW_CONSTANTS.kShootVelocity));
    }
    public CommandBase stop() {
        return this.run(()-> m_clawMotor.set(ControlMode.Velocity, CLAW_CONSTANTS.kStopVelocity));
    }
    public CommandBase hold() {
        return this.run(()-> m_clawMotor.set(ControlMode.Velocity, CLAW_CONSTANTS.kHoldVelocity));
    }
    //comment
}
