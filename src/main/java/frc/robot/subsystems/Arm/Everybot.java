package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EverybotConstants;

public class Everybot extends SubsystemBase {
    
    private CANSparkMax intake;
    String mode;
    public Everybot() {
        intake = new CANSparkMax(EverybotConstants.INTAKE_ID, MotorType.kBrushless);
        intake.setIdleMode(IdleMode.kBrake);
        mode = "cube";
    }

    // mode can only be 'cube' or 'cone'
    public void setMode(String inMode) {
        mode = inMode;
    }

    public String getMode() {
        return mode;
    }

    public void setIntakeMotor(double percent, int amps) {
        intake.set(percent);
        intake.setSmartCurrentLimit(amps);
        SmartDashboard.putNumber("intake power (%)", percent);
        SmartDashboard.putNumber("intake motor current (amps)", intake.getOutputCurrent());
        SmartDashboard.putNumber("intake motor temperature (C)", intake.getMotorTemperature());
      }
    
}
