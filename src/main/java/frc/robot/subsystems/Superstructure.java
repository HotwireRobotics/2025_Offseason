package frc.robot.subsystems;

import java.lang.annotation.Target;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

public class Superstructure extends SubsystemBase {

	public enum TargetState {
		/**
		 * Robot is offline, move to starting positons.
		 */
		STOPPED, 
		/**
		 * Transition to the subsequent state based on current conditions. (This is only a <strong>TARGET</strong> state.)
		 */
		DEFAULT, 
		SCORE_LEFT, 
		SCORE_RIGHT, 
		GET_CORAL
	}

	public TargetState targetSuperState = TargetState.STOPPED;

	public enum SystemState {
		/**
		 * Robot is offline, move to starting positons.
		 */
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
	private Intake intake;
	private Arm arm;
	
	public Superstructure(RobotContainer container) {

		this.drivetrain = container.drivetrain;
		this.intake = container.intake;
		this.arm = container.arm;

	}

	public void periodic() {handleStateTransitions(); applyStates();}

	private void applyStates() {

		switch (currentSuperState) {
			case STOPPED:
				drivetrain.targetState = DriveTrain.TargetState.IDLE;
				intake.targetState = Intake.TargetState.STOPPED;
				arm.targetState = Arm.TargetState.STOPPED;
				break;
			case NO_CORAL:
				drivetrain.targetState = DriveTrain.TargetState.TELEOP_DRIVE;
				break;
			case HOLDING_CORAL:
				drivetrain.targetState = DriveTrain.TargetState.TELEOP_DRIVE;
				break;
			case SCORING_CORAL_LEFT:
				drivetrain.targetState = DriveTrain.TargetState.FOLLOW_PATH;
				break;
			case SCORING_CORAL_RIGHT:
				drivetrain.targetState = DriveTrain.TargetState.FOLLOW_PATH;
				break;
			default:
				break;
		}
	}

	public Command setTargetState(TargetState state) {
		return new InstantCommand(() -> {targetSuperState = state;});
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