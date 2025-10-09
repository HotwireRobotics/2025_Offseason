package frc.robot.example;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

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
}
