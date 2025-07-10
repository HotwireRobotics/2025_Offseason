package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {

	public enum TargetState {
		STOPPED, 
		DEFAULT, 
		SCORE_LEFT, 
		SCORE_RIGHT, 
		GET_CORAL
	}

	public TargetState targetSuperState = TargetState.STOPPED;

	public enum SystemState {
		STOPPED,
		NO_CORAL, 
		SCORING_CORAL_LEFT,
		SCORING_CORAL_RIGHT, 
		INTAKING_CORAL, 
		HOLDING_CORAL
	}

	public SystemState currentSuperState = SystemState.STOPPED;
	public SystemState previousSuperState;

	private DriveTrain drivetrain;
	
	public Superstructure(DriveTrain drivetrain /* Include all subsystems. */) {

		this.drivetrain = drivetrain; // This is a drivetrain

	}

	@Override
	public void periodic() {handleStateTransitions(); applyStates();}

	private void applyStates() {

		switch (currentSuperState) {
			case NO_CORAL:
				// drivetrain.state = DriveTrain.TargetState.;
				break;
		}

	}

	private void handleStateTransitions() {

		previousSuperState = currentSuperState;

		switch (targetSuperState) {
			default:
				currentSuperState = SystemState.STOPPED;
				break;
			case STOPPED:
				break;
			case DEFAULT:
				// Handle open-ended states.
				currentSuperState = SystemState.NO_CORAL;
				break;
			case SCORE_RIGHT:
				// Begin moving towards nearest right pole.
				currentSuperState = SystemState.SCORING_CORAL_RIGHT;
				break;
			case SCORE_LEFT:
				// Begin moving towards nearest left pole.
				currentSuperState = SystemState.SCORING_CORAL_LEFT;
				break;
			case GET_CORAL:
				//! Fake intake present BEWARE
				currentSuperState = SystemState.HOLDING_CORAL;
				break;
		}
	}
}