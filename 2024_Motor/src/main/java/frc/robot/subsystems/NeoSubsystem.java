package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DeviceId;

public class NeoSubsystem extends SubsystemBase {
    private final CANSparkMax motor;

    public NeoSubsystem() {
        this.motor = new CANSparkMax(DeviceId.Neo.motor, MotorType.kBrushless);
        this.motor.setSmartCurrentLimit(30); // 電流限制
        this.motor.setInverted(false);  // 是否反轉
        this.motor.setIdleMode(IdleMode.kBrake); // kBrake 停止後鎖住馬達, kCoast 停止後保持慣性
    }

    public void move(double speed) {
        this.motor.set(speed * Constants.MAX_DRIVE_SPEED);
        SmartDashboard.putNumber("Neo Speed", speed * Constants.MAX_DRIVE_SPEED); // 輸出速度到 SmartDashboard 
    }

    public void stop() {
        this.motor.stopMotor();
    }
}