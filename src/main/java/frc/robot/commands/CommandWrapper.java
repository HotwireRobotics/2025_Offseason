package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class CommandWrapper extends Command {
	public Command command;
	public Executable function;

	@FunctionalInterface
	public interface Executable {
		void execute();
	}

	public CommandWrapper(Command command, Executable anonymousFunction) {
		this.command = command;
		this.function = anonymousFunction;
	}

	@Override
	public void initialize() {
		command.schedule();
	}

	@Override
	public void end(boolean interrupted) {
		function.execute();
	}
}