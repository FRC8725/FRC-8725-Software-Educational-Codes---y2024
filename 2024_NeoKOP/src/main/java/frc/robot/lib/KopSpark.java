package frc.robot.lib;

import com.revrobotics.CANSparkMax;

public class KopSpark extends CANSparkMax {
    public KopSpark(int id, boolean reverse) {
        super(id, MotorType.kBrushless);
        this.setIdleMode(IdleMode.kCoast);
        this.setInverted(reverse);
        this.setSmartCurrentLimit(30);
    }
}
