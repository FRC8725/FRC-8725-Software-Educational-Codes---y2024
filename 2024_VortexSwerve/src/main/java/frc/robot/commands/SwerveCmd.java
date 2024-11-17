package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveCmd extends Command {
	private final SwerveSubsystem swerveSubsystem;
	private final Supplier<Double> xSpeed, ySpeed, rotationSpeed;

	public SwerveCmd(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rotationSpeed) {
		this.swerveSubsystem = swerveSubsystem;
		this.xSpeed = xSpeed;
		this.ySpeed = ySpeed;
		this.rotationSpeed = rotationSpeed;
		this.addRequirements(this.swerveSubsystem);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		// The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
		this.swerveSubsystem.driveSwerve(this.xSpeed.get(), this.ySpeed.get(), this.rotationSpeed.get(), Constants.gyroField);
	}

	@Override
	public void end(boolean interrupted) {
		this.swerveSubsystem.stopModules();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
