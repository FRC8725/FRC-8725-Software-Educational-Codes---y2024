package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCmd extends Command {
	private final DriveSubsystem driveSubsystem;
	private final XboxController controller;

	public DriveCmd(DriveSubsystem driveSubsystem, XboxController controller) {
		this.driveSubsystem = driveSubsystem;
		this.controller = controller;

		addRequirements(this.driveSubsystem);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		double driveSpeed = -MathUtil.applyDeadband(this.controller.getLeftY(), 0.05) * 0.3;
		double turnSpeed = MathUtil.applyDeadband(this.controller.getRightX(), 0.05) * 0.5;
		this.driveSubsystem.setSpeed(driveSpeed + turnSpeed, driveSpeed - turnSpeed);
	}

	@Override
	public void end(boolean interrupted) {
		this.driveSubsystem.stopModules();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
