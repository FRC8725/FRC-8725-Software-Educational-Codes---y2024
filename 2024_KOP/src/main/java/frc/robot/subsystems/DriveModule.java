package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.Constants.Drive;

public class DriveModule {
    private final VictorSPX motor;

    public DriveModule(int port, boolean reverse) {
        this.motor = new VictorSPX(port);
        this.motor.enableVoltageCompensation(true);
        this.motor.configVoltageCompSaturation(15.0);
        this.motor.setInverted(reverse);
        this.motor.setNeutralMode(NeutralMode.Brake);
    }

    public void setDesiredState(double speed) {
        this.motor.set(VictorSPXControlMode.PercentOutput, speed * Drive.MAX_SPEED);
    }

    public void stop() {
        this.motor.set(VictorSPXControlMode.PercentOutput, 0.0);
    }
}
