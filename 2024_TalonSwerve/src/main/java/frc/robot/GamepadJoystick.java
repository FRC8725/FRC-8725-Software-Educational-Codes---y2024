package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.SwerveConstants;

public class GamepadJoystick extends XboxController {
    private final SlewRateLimiter translateLimiter = new SlewRateLimiter(SwerveConstants.MAX_ACCELERATION);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(SwerveConstants.MAX_ANGULAR_ACCELERATION);

    public GamepadJoystick(int port) {
        super(port);
    }
    public static final int DRIVER_PORT = 0;

    public double getXDesiredSpeed() {
        double speed = -MathUtil.applyDeadband(this.getLeftX(), Constants.DEAD_BAND);
        return this.translateLimiter.calculate(speed) * this.getBrake();
    }

    public double getYDesiredSpeed() {
        double speed = -MathUtil.applyDeadband(this.getLeftY(), Constants.DEAD_BAND);
        return this.translateLimiter.calculate(speed) * this.getBrake();
    }

    public double getRotationSpeed() {
        double speed = -MathUtil.applyDeadband(this.getRightX(), Constants.DEAD_BAND);
        return this.rotationLimiter.calculate(speed) * this.getBrake();
    }

    public double getBrake() {
        return 1.0 - MathUtil.applyDeadband(this.getLeftTriggerAxis(), Constants.DEAD_BAND);
    }
}
