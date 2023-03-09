package frc.robot.subsystems.Arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CLAW_CONSTANTS;



public class Claw extends SubsystemBase {
    private TalonSRX m_clawMotor;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;  
    private boolean m_holding = false;
 

    
    public Claw() {
        m_clawMotor = new TalonSRX(CLAW_CONSTANTS.CLAW_ID);
        m_clawMotor.configFactoryDefault();
        m_clawMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, CLAW_CONSTANTS.kPIDLoopIdx, CLAW_CONSTANTS.kTimeoutMs);
        m_clawMotor.setSensorPhase(true);

        // neutral mode
        m_clawMotor.setNeutralMode(NeutralMode.Brake);

        /* Config the peak and nominal outputs */
        m_clawMotor.configNominalOutputForward(0, CLAW_CONSTANTS.kTimeoutMs);
        m_clawMotor.configNominalOutputReverse(0, CLAW_CONSTANTS.kTimeoutMs);
        m_clawMotor.configPeakOutputForward(1, CLAW_CONSTANTS.kTimeoutMs);
        m_clawMotor.configPeakOutputReverse(-1, CLAW_CONSTANTS.kTimeoutMs);

        /* Config the Velocity closed loop gains in slot0 */
        m_clawMotor.config_kF(CLAW_CONSTANTS.kPIDLoopIdx, CLAW_CONSTANTS.kGains_Velocity.kF, CLAW_CONSTANTS.kTimeoutMs);
        m_clawMotor.config_kP(CLAW_CONSTANTS.kPIDLoopIdx, CLAW_CONSTANTS.kGains_Velocity.kP, CLAW_CONSTANTS.kTimeoutMs);
        m_clawMotor.config_kI(CLAW_CONSTANTS.kPIDLoopIdx, CLAW_CONSTANTS.kGains_Velocity.kI, CLAW_CONSTANTS.kTimeoutMs);
        m_clawMotor.config_kD(CLAW_CONSTANTS.kPIDLoopIdx, CLAW_CONSTANTS.kGains_Velocity.kD, CLAW_CONSTANTS.kTimeoutMs);

        m_clawMotor.setSensorPhase(false); // invert encoder to match motor

        ShuffleboardTab shuffTab = Shuffleboard.getTab("Claw");
        shuffTab.addDouble("Motor V", () -> m_clawMotor.getSelectedSensorVelocity());
        shuffTab.addDouble("Motor Pos", () -> m_clawMotor.getSelectedSensorPosition());
        shuffTab.addDouble("Motor Stator I", () -> getClawStatorCurrent());
        shuffTab.addDouble("Motor Supply I", () -> getClawSupplyCurrent());
        shuffTab.addBoolean("Stalled", () -> checkStalledCondition());
        shuffTab.addBoolean("Holding", () -> m_holding);

    }

    public void runClawClosedLoop (double velocity) {
        m_holding = false;
        m_clawMotor.set(ControlMode.Velocity, velocity);
    }

    // Run these from RobotContainer like this:
    //      new JoystickButton(m_driverController, Button.kA.value)
    //          .whileTrue(m_claw.in());

    private double getClawStatorCurrent () {
        return m_clawMotor.getStatorCurrent();
    }

    private double getClawSupplyCurrent () {
        return m_clawMotor.getSupplyCurrent();
    }
    public boolean checkStalledCondition() {
        return (getClawSupplyCurrent() > CLAW_CONSTANTS.kStallCurrent);
    }

    public void prepareToHold() {
        m_clawMotor.setSelectedSensorPosition(0.0);
        m_holding = false;
    }

    public void hold(double position) {
        m_clawMotor.set(ControlMode.Position, position);
        m_holding = true;
    }

    public CommandBase in() {
        return this.run(()-> runClawClosedLoop(CLAW_CONSTANTS.kInVelocity)); // testing on celestial was with 10000 to 30000
     }
     public CommandBase coneOut() {
         return this.run(()-> runClawClosedLoop(CLAW_CONSTANTS.kConeOutVelocity));
     }
     public CommandBase cubeOut() {
         return this.run(()-> runClawClosedLoop(CLAW_CONSTANTS.kCubeOutVelocity));
     }
     public CommandBase shoot() {
         return this.run(()-> runClawClosedLoop(CLAW_CONSTANTS.kShootVelocity));
     }
     public CommandBase stop() {
         return this.run(()-> runClawClosedLoop(CLAW_CONSTANTS.kStopVelocity));
     }
    //  public CommandBase hold() {
    //      return this.run(()-> runClawClosedLoop(CLAW_CONSTANTS.kHoldVelocity));
    //  }


 
}
