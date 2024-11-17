package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeviceId.Vortex;
import frc.robot.lib.IDashboardProvider;
import frc.robot.DeviceId.Encoder;
import frc.robot.Constants.MotorReverse;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.DriveEncoderReverse;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase implements IDashboardProvider {
    // Create four swerve module.
    private final SwerveModule frontLeft = new SwerveModule(
        Vortex.frontLeftDrive,
        Vortex.frontLeftTurn,
        Encoder.frontLeft,
        MotorReverse.FRONT_LEFT_DRIVE,
        MotorReverse.FRONT_LEFT_TURN,
        DriveEncoderReverse.FRONT_LEFT,
        "frontLeft"
    );
    private final SwerveModule frontRight = new SwerveModule(
        Vortex.frontRightDrive,
        Vortex.frontRightTurn,
        Encoder.frontRight,
        MotorReverse.FRONT_RIGHT_DRIVE,
        MotorReverse.FRONT_RIGHT_TURN,
        DriveEncoderReverse.FRONT_RIGHT,
        "frontRight"
    );
    private final SwerveModule backLeft = new SwerveModule(
        Vortex.backLeftDrive,
        Vortex.backLeftTurn,
        Encoder.backLeft,
        MotorReverse.BACK_LEFT_DRIVE,
        MotorReverse.BACK_LEFT_TURN,
        DriveEncoderReverse.BACK_LEFT,
        "backLeft"
    );
    private final SwerveModule backRight = new SwerveModule(
        Vortex.backRightDrive,
        Vortex.backRightTurn,
        Encoder.backRight,
        MotorReverse.BACK_RIGHT_DRIVE,
        MotorReverse.BACK_RIGHT_TURN,
        DriveEncoderReverse.BACK_RIGHT,
        "backRight"
    );
    // Create gyro sensor.
    private final Pigeon2 gyro = new Pigeon2(Encoder.pigeon);
    // Odometry class for tracking robot pose
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
        Constants.swerveDriveKinematics, this.gyro.getRotation2d(), this.getModulePosition()
    );

    public SwerveSubsystem() {
        this.registerDashboard();
        // Delay 1000 ms.
        try {
            Thread.sleep(1000); // ms
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        this.gyro.reset();
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block.
        this.odometry.update(this.gyro.getRotation2d(), getModulePosition());
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void driveSwerve(double xSpeed, double ySpeed, double rotation, boolean field) {
        SwerveModuleState[] state = Constants.swerveDriveKinematics.toSwerveModuleStates(field ? 
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, this.gyro.getRotation2d()) :
            new ChassisSpeeds(xSpeed, ySpeed, rotation)
        );
        this.setModuleState(state);
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleState(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_SPEED);
        this.frontLeft.setDesiredState(states[0]);
        this.frontRight.setDesiredState(states[1]);
        this.backLeft.setDesiredState(states[2]);
        this.backRight.setDesiredState(states[3]);
    }

    /**
     * Returns the current state of the swerve.
     *
     * @return The current state of the swerve.
     */
    public SwerveModuleState[] getModuleState() {
        return new SwerveModuleState[] {
            this.frontLeft.getState(),
            this.frontRight.getState(),
            this.backLeft.getState(),
            this.backRight.getState()
        };
    }

    /**
     * Returns the current position of the swerve.
     *
     * @return The current position of the swerve.
     */
    public SwerveModulePosition[] getModulePosition() {
        return new SwerveModulePosition[] {
            this.frontLeft.getPosition(),
            this.frontRight.getPosition(),
            this.backLeft.getPosition(),
            this.backRight.getPosition()
        };
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return this.odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetPose(Pose2d pose) {
        this.odometry.resetPosition(this.gyro.getRotation2d(), this.getModulePosition(), pose);
    }

    /**
     * Returns the chassis speed of the swerve
     * 
     * @return The chassis speed of the swerve
     */
    public ChassisSpeeds getSpeeds() {
        return Constants.swerveDriveKinematics.toChassisSpeeds(this.getModuleState());
    }

    // Stop swerve.
    public void stopModules() {
        this.frontLeft.stop();
        this.frontRight.stop();
        this.backLeft.stop();
        this.backRight.stop();
    }

    @Override
    public void putDashboard() {
    }
}