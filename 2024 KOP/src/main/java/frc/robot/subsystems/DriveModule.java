package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class DriveModule {
    private final VictorSPX motor;

    public DriveModule(int port, boolean reverse) {
        this.motor = new VictorSPX(port);
        this.motor.setInverted(reverse);
        this.motor.setNeutralMode(NeutralMode.Brake);
    }

    public void setDesiredState(double speed) {
        this.motor.set(VictorSPXControlMode.PercentOutput, speed * 0.7);
    }

    public void stopModule() {
        this.motor.set(VictorSPXControlMode.PercentOutput, 0.0);
    }
}
