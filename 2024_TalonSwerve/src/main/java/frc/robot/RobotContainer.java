package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SwerveDriveCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
	private final GamepadJoystick driverJoystick = new GamepadJoystick(GamepadJoystick.DRIVER_PORT);
	private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
	private final SwerveDriveCmd swerveDriveCmd = new SwerveDriveCmd(swerveSubsystem, driverJoystick);

	public RobotContainer() {
		this.swerveSubsystem.setDefaultCommand(this.swerveDriveCmd);
	}

	public Command getAutonomousCommand() {
		return null;
	}
}
