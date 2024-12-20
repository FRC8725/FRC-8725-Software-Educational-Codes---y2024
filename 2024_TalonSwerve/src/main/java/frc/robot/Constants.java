package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
	public static final class SwerveConstants {
		public static final double TRACK_WIDTH = 0.61; // 寬
		public static final double WHEEL_BASE = 0.61; // 長
		public static final double WHEEL_RADIUS = 0.0508; // 輪子半徑
		
		public static final double MAX_SPEED = 3.0; // 最大速度m/s
		public static final double MAX_ACCELERATION = 3.0; // 最大加速度m/s^2
		public static final double MAX_ANGULAR_ACCELERATION = 3.0;
		public static final double DRIVE_GEAR_RATIO = 1.0 / 8.14; // 齒輪比
		public static final double TURN_GEAR_RATIO = 7.0 / 150.0;
		public static final double MAX_VOLTAGE = 12.0;

		public static final double DRIVE_VELOCITY_CONVERSION_FACTOR = (1.0 / DRIVE_GEAR_RATIO / 2048) * WHEEL_RADIUS * Math.PI * 10;
		public static final double DRIVE_POSITION_CONVERSION_FACTOR = (1.0 / DRIVE_GEAR_RATIO / 2048) * WHEEL_RADIUS * Math.PI;
	}
	
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

	public static final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(Constants.SwerveConstants.WHEEL_BASE / 2, Constants.SwerveConstants.TRACK_WIDTH / 2),
        new Translation2d(Constants.SwerveConstants.WHEEL_BASE / 2, -Constants.SwerveConstants.TRACK_WIDTH / 2),
        new Translation2d(-Constants.SwerveConstants.WHEEL_BASE / 2, Constants.SwerveConstants.TRACK_WIDTH / 2),
        new Translation2d(-Constants.SwerveConstants.WHEEL_BASE / 2, -Constants.SwerveConstants.TRACK_WIDTH / 2)
    );

    public static final double DEAD_BAND = 0.05;
	public static final boolean gyroField = true;
}
