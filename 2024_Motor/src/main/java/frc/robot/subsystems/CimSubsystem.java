package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DeviceId;

public class CimSubsystem extends SubsystemBase {
    private final VictorSPX motor;

    public CimSubsystem() {
        this.motor = new VictorSPX(DeviceId.CIM.motor);
        this.motor.enableVoltageCompensation(true); // 是否啟用電壓補償
        this.motor.configVoltageCompSaturation(15.0); // 電壓輸出百分比
        this.motor.setNeutralMode(NeutralMode.Brake); // Brake 停止後鎖住馬達, Coast 停止後保持慣性
        this.motor.setInverted(false); // 是否反轉
    }

    public void move(double speed) {
        this.motor.set(VictorSPXControlMode.PercentOutput, speed * Constants.MAX_DRIVE_SPEED);
        SmartDashboard.putNumber("CIM Speed", speed * Constants.MAX_DRIVE_SPEED); // 輸出速度到 SmartDashboard 
    }

    public void stop() {
        this.motor.set(VictorSPXControlMode.PercentOutput, 0.0);
    }
}