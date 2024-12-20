package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;
import frc.robot.lib.IDashboardProvider;
import frc.robot.lib.SwerveTalon;

public class SwerveModule implements IDashboardProvider {
    private final SwerveTalon driveMotor;
    private final SwerveTalon turnMotor;

    private final CANcoder turnEncoder;

    private final PIDController turnPidController;

    private final String motorName;
    private double driveOutput;
    private double turnOutput;

    public SwerveModule(
        int driveMotorPort, int turnMotorPort, int turnEncoderPort,
        boolean driveMotorReverse, boolean turnMotorReverse,
        String motorName
    ){
        this.registerDashboard();

        this.driveMotor = new SwerveTalon(driveMotorPort, driveMotorReverse, SwerveConstants.DRIVE_GEAR_RATIO);
        this.turnMotor = new SwerveTalon(turnMotorPort, turnMotorReverse, SwerveConstants.TURN_GEAR_RATIO);

        this.turnEncoder = new CANcoder(turnEncoderPort);

        this.turnPidController = new PIDController(0.009, 0, 0);
        this.turnPidController.enableContinuousInput(-180, 180);

        this.motorName = motorName;
        this.resetEncoders();
    }

    public void resetEncoders() {
        this.driveMotor.setPosition(0);
        this.turnMotor.setPosition(this.getTurnPosition());
    }

    public double getTurnPosition() {
        double value = Units.rotationsToDegrees(this.turnEncoder.getAbsolutePosition().getValue());
        value %= 360.0;
        return value > 180 ? value - 360 : value;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            this.driveMotor.getMotorVelocity(),
            Rotation2d.fromDegrees(this.getTurnPosition())
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            this.driveMotor.getMotorPosition(),
            Rotation2d.fromDegrees(this.getTurnPosition())
        );
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            this.stop();
            return;
        }
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, this.getState().angle);

        this.driveOutput = state.speedMetersPerSecond / SwerveConstants.MAX_SPEED;
        this.turnOutput = this.turnPidController.calculate(
            this.getTurnPosition(), state.angle.getDegrees()
        );

        this.driveMotor.set(this.driveOutput);
        this.turnMotor.set(this.turnOutput);
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber(this.motorName + " Drive Vel", this.getState().speedMetersPerSecond);
        SmartDashboard.putNumber(this.motorName + " Turn Pos", this.getTurnPosition());
    }

    public void stop() {
        this.driveMotor.set(0.0);
        this.turnMotor.set(0.0);
    }
}
