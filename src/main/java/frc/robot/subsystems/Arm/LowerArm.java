package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LowerArmConstants;

public class LowerArm {
    private final CANSparkMax m_elbow;

    public LowerArm() {
        m_elbow = new CANSparkMax(LowerArmConstants.LOWER_ARM_MOTOR_ID, MotorType.kBrushless);
        m_elbow.restoreFactoryDefaults();
        m_elbow.setSmartCurrentLimit(20);
        m_elbow.setIdleMode(IdleMode.kBrake);
        m_elbow.burnFlash();
    }

    // public CommandBase moveToRelativeAngle() {
    //     return this.run(() -> m_elbow.
    // }
}


