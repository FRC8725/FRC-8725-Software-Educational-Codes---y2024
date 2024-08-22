package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCmd;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
	private final XboxController driverJoystick = new XboxController(GamepadJoystick.CONTROLLER_PORT);
	private final DriveSubsystem driveSubsystem = new DriveSubsystem();
	private final DriveCmd driveCmd = new DriveCmd(driveSubsystem, driverJoystick);

	public RobotContainer() {
		this.driveSubsystem.setDefaultCommand(this.driveCmd);
	}

	public Command getAutonomousCommand() {
		return null;
	}
}
