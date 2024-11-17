package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
	public static final class SwerveConstants {
		/**
		 * The distance from the center point of a Swerve Module to another Swerve Module.
		 * Units: Meters
		 */
		public static final double TRACK_WIDTH = 0.66;
		public static final double TRACK_LENGTH = 0.66;
		public static final double WHEEL_RADIUS = 0.0508;
		
		// Limit Swerve maximum speed.
		public static final double MAX_SPEED = 3.0;
		public static final double MAX_ACCLERATION = 1.0;
		public static final double MAX_ANGULAR_ACCLERATION = 9.0;
		// Drive Motor gear ratio.
		public static final double DRIVE_GEAR_RATIO = 57.0 / 7.0;
		// Limit motor voltage.
		public static final int MAX_VOLTAGE = 20;

		/**
		 * Motor setting gear ratio.
		 * Velocity Units: meter per seconds.
		 * Position Units: degrees.
		 */
		public static final double DRIVE_VELOCITY_CONVERSION_FACTOR = WHEEL_RADIUS * 2 / DRIVE_GEAR_RATIO * Math.PI / 60;
		public static final double DRIVE_POSITION_CONVERSION_FACTOR = WHEEL_RADIUS * 2 / DRIVE_GEAR_RATIO * Math.PI;
	}
	
	/**
	 * Set the motor to rotate forward and reverse to ensure
	 * that the value is positive when Swerve moves forward.
	 */
	public static final class MotorReverse {
		public static final boolean FRONT_LEFT_DRIVE = true;
		public static final boolean FRONT_RIGHT_DRIVE = false;
		public static final boolean BACK_LEFT_DRIVE = true;
		public static final boolean BACK_RIGHT_DRIVE = false;

		public static final boolean FRONT_LEFT_TURN = true;
		public static final boolean FRONT_RIGHT_TURN = true;
		public static final boolean BACK_LEFT_TURN = true;
		public static final boolean BACK_RIGHT_TURN = true;
	}

	/**
	 * Set the Encoder Reverse to ensure
	 * that the value of Swerve is positive when moving forward.
	 */
	public static final class DriveEncoderReverse {
		public static final boolean FRONT_LEFT = true;
		public static final boolean FRONT_RIGHT = true;
		public static final boolean BACK_LEFT = true;
		public static final boolean BACK_RIGHT = true;
	}

	// From the robot center to the coordinates of each Module.
	public static final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(Constants.SwerveConstants.TRACK_LENGTH / 2, Constants.SwerveConstants.TRACK_WIDTH / 2),
        new Translation2d(Constants.SwerveConstants.TRACK_LENGTH / 2, -Constants.SwerveConstants.TRACK_WIDTH / 2),
        new Translation2d(-Constants.SwerveConstants.TRACK_LENGTH / 2, Constants.SwerveConstants.TRACK_WIDTH / 2),
        new Translation2d(-Constants.SwerveConstants.TRACK_LENGTH / 2, -Constants.SwerveConstants.TRACK_WIDTH / 2)
    );

	// When the output value is less than 0.05, it is regarded as 0.
    public static final double DEAD_BAND = 0.05;
	public static final boolean gyroField = true;
}
