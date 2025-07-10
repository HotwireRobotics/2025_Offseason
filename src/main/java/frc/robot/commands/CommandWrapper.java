package frc.robot.commands;

import java.lang.reflect.Executable;

import edu.wpi.first.wpilibj2.command.Command;

public class CommandWrapper extends Command {
	public Command command;
	public Executable endingAnonymousFunction;
	public Executable beginningAnonymousFunction;

	@FunctionalInterface
	public interface Executable {
		void execute();
	}

	public CommandWrapper(Command command, Executable endingAnonymousFunction, Executable beginningAnonymousFunction) {
		this.command = command;
		this.endingAnonymousFunction = endingAnonymousFunction;
		this.beginningAnonymousFunction = beginningAnonymousFunction;
	}

	@Override
	public void initialize() {
		beginningAnonymousFunction.execute();
		command.schedule();
	}

	@Override
	public void end(boolean interrupted) {
		endingAnonymousFunction.execute();
	}
}