package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;
import frc.robot.lib.IDashboardProvider;

public class SwerveModule implements IDashboardProvider {
    private final CANSparkFlex driveMotor;
    private final CANSparkFlex turnMotor;

    private final RelativeEncoder driveEncoder;
    private final CANcoder turnEncoder;

    private final PIDController turnPid;

    private final Boolean driveEncoderReverse;
    private final String motorName;
    private double driveOutput;
    private double turnOutput;

    /**
     * Constructs a SwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * Swerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */
    public SwerveModule(
        int drivePort, int turnPort, int encoderPort,
        boolean driveReverse, boolean turnReverse, boolean driveEncoderReverse,
        String motorName
    ) {
        this.registerDashboard();
        this.driveMotor = new CANSparkFlex(drivePort, MotorType.kBrushless);
        this.turnMotor = new CANSparkFlex(turnPort, MotorType.kBrushless);

        this.driveEncoder = this.driveMotor.getEncoder();
        this.turnEncoder = new CANcoder(encoderPort);

        // Reset Driving and Turning motor.
        this.driveMotor.restoreFactoryDefaults();
        this.turnMotor.restoreFactoryDefaults();

        this.driveMotor.setInverted(driveReverse);
        this.driveMotor.setIdleMode(IdleMode.kBrake);
        this.turnMotor.setInverted(turnReverse);
        this.turnMotor.setIdleMode(IdleMode.kBrake);
        
        // Convert motor angle to wheel angle in degrees.
        this.driveEncoder.setPositionConversionFactor(SwerveConstants.DRIVE_POSITION_CONVERSION_FACTOR);
        // Convert the number of motor turns to the number of wheel turns in m/s.
        this.driveEncoder.setVelocityConversionFactor(SwerveConstants.DRIVE_VELOCITY_CONVERSION_FACTOR);

        this.turnPid = new PIDController(0.0065, 0.00005, 0.0);
        // Angle use -180 to 180, Radian use -Math.PI to Math.PI.
        this.turnPid.enableContinuousInput(-180, 180);

        this.driveEncoderReverse = driveEncoderReverse;
        this.motorName = motorName;
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            this.driveEncoder.getVelocity(),
            Rotation2d.fromDegrees(this.getTurningEncoderPosition())
        );
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            this.driveEncoder.getPosition(),
            Rotation2d.fromDegrees(this.getTurningEncoderPosition())
        );
    }

    // Adjust motor positive and negative values.
    public double getDriveEncoderPosition() {
        return this.driveEncoder.getPosition() * (this.driveEncoderReverse ? 1 : -1);
    }

    // Adjust motor positive and negative values.
    public double getDriveEncoderVelocity() {
        return this.driveEncoder.getVelocity() * (this.driveEncoderReverse ? 1 : -1);
    }

    // Output the Encoder as Degrees.
    public double getTurningEncoderPosition() {
        double value = Units.rotationsToDegrees(this.turnEncoder.getAbsolutePosition().getValue());
        value %= 360.0;
        return value > 180 ? value - 360 : value;
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // If the output is less than 0.001 (m/s), it is regarded as 0.
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            this.stop();
            return;
        }
        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, this.getState().angle);

        // Calculate output.
        this.driveOutput = state.speedMetersPerSecond / SwerveConstants.MAX_SPEED;
        this.turnOutput = this.turnPid.calculate(this.getState().angle.getDegrees(), state.angle.getDegrees());

        this.driveMotor.set(this.driveOutput);
        this.turnMotor.set(this.turnOutput);
    }

    // Stop module.
    public void stop() {
        this.driveMotor.stopMotor();
        this.turnMotor.stopMotor();
    }

    // Put Drive Velocity and Turn Position to SmartDashboard.
    @Override
    public void putDashboard() {
        SmartDashboard.putNumber(motorName + " Drive Vel", this.getState().speedMetersPerSecond);
        SmartDashboard.putNumber(motorName + "Turn Pos", this.getTurningEncoderPosition());
    }
}
