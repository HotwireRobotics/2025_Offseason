package frc.robot.subsystems;

import java.lang.annotation.Target;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
		SCORE_DOWN_LEFT,
		SCORE_UP_LEFT,
		SCORE_DOWN_RIGHT,
		SCORE_UP_RIGHT,
		/**
		 * Intake coral gamepiece.
		 */
		INTAKE
	}

	public TargetState targetSuperState = TargetState.STOPPED;

	public enum SystemState {
		/**
		 * Robot is offline, move to starting positons.
		 */
		STOPPED,
		/**
		 * Robot is in teleop; the driver is in control.
		 */
		HOME,
		SCORING_DOWN_LEFT,
		SCORING_UP_LEFT,
		SCORING_DOWN_RIGHT,
		SCORING_UP_RIGHT,
		/**
		 * Robot is intaking a coral gamepiece.
		 */
		INTAKING
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

	private void handleStateTransitions() {

		previousSuperState = currentSuperState;

		switch (targetSuperState) {
			case STOPPED:
				break;
			case DEFAULT:
				currentSuperState = SystemState.HOME;
				break;
			case INTAKE:
				currentSuperState = SystemState.INTAKING;
				break;
			case SCORE_DOWN_RIGHT:
				break;
			default:
				currentSuperState = SystemState.STOPPED;
				break;
		}
	}

	private void applyStates() {

		switch (currentSuperState) {
			case STOPPED:
				drivetrain.targetState = DriveTrain.TargetState.IDLE;
				intake.targetState = Intake.TargetState.STOPPED;
				arm.targetState = Arm.TargetState.STOP;
				break;
			case HOME:
				drivetrain.targetState = DriveTrain.TargetState.TELEOP_DRIVE;
				intake.targetState = Intake.TargetState.HOLD;
				arm.targetState = Arm.TargetState.HOME;
				break;
			case INTAKING:
				intake.targetState = Intake.TargetState.COLLECT;
				arm.targetState = Arm.TargetState.INTAKE;
				break;
			default:
				break;
		}
	}
}