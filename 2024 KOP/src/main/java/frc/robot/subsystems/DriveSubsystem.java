package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    private final DriveModule frontLeft;
    private final DriveModule backLeft;
    private final DriveModule frontRight;
    private final DriveModule backRight;

    public DriveSubsystem() {
        this.frontLeft = new DriveModule(1, false);
        this.backLeft = new DriveModule(2, false);
        this.frontRight = new DriveModule(3, true);
        this.backRight = new DriveModule(4, true);
    }

    public void setSpeed(double leftSpeed, double rightSpeed) {
        this.frontLeft.setDesiredState(leftSpeed);
        this.backLeft.setDesiredState(leftSpeed);
        this.frontRight.setDesiredState(rightSpeed);
        this.backRight.setDesiredState(rightSpeed);
    }

    public void stopModules() {
        this.frontLeft.stopModule();
        this.frontRight.stopModule();
        this.backLeft.stopModule();
        this.frontRight.stopModule();
    }
}
