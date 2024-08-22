package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DeviceId;

public class TalonSubsystem extends SubsystemBase {
    private final TalonFX motor;

    public TalonSubsystem() {
        this.motor = new TalonFX(DeviceId.Talon.motor);
        this.motor.setInverted(false); // 是否反轉
        this.motor.setNeutralMode(NeutralModeValue.Brake); // Brake 停止後鎖住馬達, Coast 停止後保持慣性
    }

    public void move(double speed) {
        SmartDashboard.putNumber("Talon Speed", speed * Constants.MAX_DRIVE_SPEED);
        this.motor.set(speed * Constants.MAX_DRIVE_SPEED);
    }

    public void stop() {
        this.motor.stopMotor();
    }
}
