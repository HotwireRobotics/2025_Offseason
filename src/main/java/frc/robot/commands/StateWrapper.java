package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class StateWrapper extends Command {
	public DriveTrain drivetrain;
	public Command command;
	public FunctionalInterface function;

	public StateWrapper(DriveTrain drivetrain, Command command, FunctionalInterface anonymousFunction) {
		this.drivetrain = drivetrain;
		this.command = command;
		this.function = anonymousFunction;
	}

	public void initialize() {
		command.schedule();
	}

	public void end(boolean interrupted) {
		// function //! yo execute this FunctionalInterface
		System.out.println(drivetrain.state);
	}
}

