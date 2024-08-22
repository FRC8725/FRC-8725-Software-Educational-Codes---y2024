package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.MotorCmd;
import frc.robot.subsystems.CimSubsystem;
import frc.robot.subsystems.NeoSubsystem;
import frc.robot.subsystems.TalonSubsystem;

public class RobotContainer {
	private final GamepadJoystick joystick = new GamepadJoystick(GamepadJoystick.CONTROLLER_PORT);

	private final CimSubsystem cimSubsystem = new CimSubsystem();
	private final NeoSubsystem neoSubsystem = new NeoSubsystem();
	private final TalonSubsystem talonSubsystem = new TalonSubsystem();

	private final MotorCmd motorCmd = new MotorCmd(cimSubsystem, neoSubsystem, talonSubsystem, joystick);

	public RobotContainer() {
		this.cimSubsystem.setDefaultCommand(this.motorCmd);
		this.neoSubsystem.setDefaultCommand(this.motorCmd);
		this.talonSubsystem.setDefaultCommand(this.motorCmd);
		this.configureBindings();
	}

	private void configureBindings() {
        // 按鈕綁定指令
    }

	public Command getAutonomousCommand() {
		// 自動階段
		return null;
	}
}
