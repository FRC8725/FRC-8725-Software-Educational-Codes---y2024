package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.KopSpark;

public class DriveMotorModule {
    private final KopSpark frontSpark;
    private final KopSpark backSpark;

    public DriveMotorModule(int frontSparkId, int backSpark, boolean frontReverse, boolean backReverse) {
        this.frontSpark = new KopSpark(frontSparkId, frontReverse);
        this.backSpark = new KopSpark(backSpark, backReverse);
    }

    public void setDesiredState(double speed) {
        this.frontSpark.set(speed);
        this.backSpark.set(speed);
    }

    public void getModuleSpeed(String name) {
        SmartDashboard.putNumber(name + " front", this.frontSpark.getEncoder().getVelocity());
        SmartDashboard.putNumber(name + " back", this.backSpark.getEncoder().getVelocity());
    }

    public void stop() {
        this.frontSpark.stopMotor();
        this.backSpark.stopMotor();
    }
}
