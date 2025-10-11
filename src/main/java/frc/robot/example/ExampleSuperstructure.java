package frc.robot.example;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSuperstructure extends SubsystemBase {
	
	//* ----- Initialization -----

	ExampleRobotContainer container;

	/**
	 * Constructor for the subsystem.
	 */
	public ExampleSuperstructure(ExampleRobotContainer container) {
		this.container = container;
	}

	//* ----- Periodic -----

	/**
	 * Periodic method for the subsystem.
	 */
	@Override
	public void periodic() {
		
	}

	//* ----- State Management -----
	
	/**
	 * Subsystem state options.
	 */
	public enum State {
		STOPPED,
		TELEOP,
		AUTO,
		PATHFINDING,		
	}

	/** 
	 * Active subsystem state.
	*/
	private State state = State.STOPPED;

	/**
	 * Returns the active subsystem state.	
	 *
	 * @return <code>state</code>
	 */
	public State getSysState() {
		return state;
	}

	/**
	 * Sets the active subsystem state.
	 */
	public void setSysState(State newState) {
		state = newState;
	}

	private Command setSuperState(State newState) {
		return new InstantCommand(() -> setSysState(State.PATHFINDING));
	}

	public Command Score(ExampleDriveTrain.ScoringPosition position) {
		return (
					 setSuperState(State.PATHFINDING)
			.andThen(container.drivetrain.new Navigate(position)
			.andThen(new ConditionalCommand(
						 container.arm.new ArmToL3()
				.andThen(new InstantCommand(() -> {System.out.println("Missing Intake");}))
				.andThen(container.arm.new ArmToFloor()),

						 container.arm.new ArmToL2()
				.andThen(new InstantCommand(() -> {System.out.println("Missing Intake");}))
				.andThen(container.arm.new ArmToFloor()),

				() -> position == ExampleDriveTrain.ScoringPosition.RIGHT_L3
				||    position == ExampleDriveTrain.ScoringPosition.LEFT_L3
			))
			.andThen(setSuperState(State.TELEOP))
		));
	}
}
