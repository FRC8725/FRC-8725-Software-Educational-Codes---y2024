package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SwerveCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
	private final GamepadJoystick joystick = new GamepadJoystick(GamepadJoystick.CONTROLLER_PORT);
	private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
	private final SwerveCmd swerveCmd = new SwerveCmd(
		swerveSubsystem, joystick::getXDesiredSpeed, joystick::getYDesiredSpeed, joystick::getRotatonSpeed
	);

	public RobotContainer() {
		this.swerveSubsystem.setDefaultCommand(this.swerveCmd);
	}

	public Command getAutonomousCommand() {
		return null;
	}
}
