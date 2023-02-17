package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CLAW_CONSTANTS;



public class Claw extends SubsystemBase {
    private CANSparkMax m_clawMotor;
    private SparkMaxPIDController m_pidController;
    private RelativeEncoder m_encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;  
    double rotations = CLAW_CONSTANTS.HOLD_ROTATIONS;  

    
    public Claw() {
        m_clawMotor = new CANSparkMax(CLAW_CONSTANTS.CLAW_ID, MotorType.kBrushless);
        m_clawMotor.restoreFactoryDefaults();
        m_pidController = m_clawMotor.getPIDController();
        m_encoder = m_clawMotor.getEncoder();

        // PID coefficients
        kP = 0.1; 
        kI = 1e-4;
        kD = 1; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;

        // set PID coefficients
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    }

    public CommandBase in() {
       return this.run(()-> m_clawMotor.set(0.7));
    }
    public CommandBase coneOut() {
        return this.run(()-> m_clawMotor.set(-0.3));
    }
    public CommandBase cubeOut() {
        return this.run(()-> m_clawMotor.set(-0.5));
    }
    public CommandBase shoot() {
        return this.run(()-> m_clawMotor.set(-1));
    }
    public CommandBase stop() {
        return this.run(()-> m_clawMotor.set(0));
    }
    public CommandBase hold() {
        return this.run(()-> m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition));
    }
    //comment
}
