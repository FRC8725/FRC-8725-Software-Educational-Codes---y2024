package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CimSubsystem;
import frc.robot.subsystems.NeoSubsystem;
import frc.robot.subsystems.TalonSubsystem;

public class MotorCmd extends Command {
	private final CimSubsystem cimSubsystem;
	private final NeoSubsystem neoSubsystem;
	private final TalonSubsystem talonSubsystem;
	private final XboxController controller;

	public MotorCmd(CimSubsystem cimSubsystem, NeoSubsystem neoSubsytem, TalonSubsystem talonSubsystem, XboxController controller) {
		this.cimSubsystem = cimSubsystem;
		this.neoSubsystem = neoSubsytem;
		this.talonSubsystem = talonSubsystem;
		this.controller = controller;
		this.addRequirements(this.cimSubsystem, this.neoSubsystem, this.talonSubsystem);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		double speed = this.controller.getLeftY();
        this.cimSubsystem.move(speed);
		this.neoSubsystem.move(speed);
		this.talonSubsystem.move(speed);
	}

	@Override
	public void end(boolean interrupted) {
		this.cimSubsystem.stop();
		this.neoSubsystem.stop();
		this.talonSubsystem.stop();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
