package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorReverse;
import frc.robot.DeviceId.DriveMotor;

public class DriveSubsystem extends SubsystemBase {
    private final DriveModule frontLeft;
    private final DriveModule backLeft;
    private final DriveModule frontRight;
    private final DriveModule backRight;

    public DriveSubsystem() {
        this.frontLeft = new DriveModule(DriveMotor.FRONT_LEFT, MotorReverse.FRONT_LEFT);
        this.backLeft = new DriveModule(DriveMotor.BACK_LEFT, MotorReverse.BACK_LEFT);
        this.frontRight = new DriveModule(DriveMotor.FRONT_RIGHT, MotorReverse.FRONT_RIGHT);
        this.backRight = new DriveModule(DriveMotor.BACK_RIGHT, MotorReverse.BACK_RIGHT);
    }

    public void move(double leftSpeed, double rightSpeed) {
        this.frontLeft.setDesiredState(leftSpeed);
        this.backLeft.setDesiredState(leftSpeed);
        this.frontRight.setDesiredState(rightSpeed);
        this.backRight.setDesiredState(rightSpeed);
    }

    public void stopModules() {
        this.frontLeft.stop();
        this.frontRight.stop();
        this.backLeft.stop();
        this.frontRight.stop();
    }
}
