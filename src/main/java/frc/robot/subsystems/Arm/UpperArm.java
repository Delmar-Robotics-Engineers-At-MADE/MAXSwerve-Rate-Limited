package frc.robot.subsystems.Arm;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.UpperArmConstants;


public class UpperArm extends SubsystemBase {
    private final WPI_TalonFX upperArmMotor;
    
    public UpperArm() {
        upperArmMotor = new WPI_TalonFX(UpperArmConstants.UPPER_ARM_MOTOR_ID);
    }
}