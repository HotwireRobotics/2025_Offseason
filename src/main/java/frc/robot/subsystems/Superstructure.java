package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import java.lang.annotation.Target;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
		// Move arm and intake into scoring position.
		SCORE_DOWN_LEFT,
		SCORE_UP_LEFT,
		SCORE_DOWN_RIGHT,
		SCORE_UP_RIGHT,

		// Navigate to nearest branch.
		NAVIGATE_DOWN_LEFT,
		NAVIGATE_UP_LEFT,
		NAVIGATE_DOWN_RIGHT,
		NAVIGATE_UP_RIGHT,
		/**
		 * Lower intake to floor level.
		 */
		LOWER,
		/**
		 * Lower intake to floor level and intake coral gamepiece.
		 */
		INTAKE,
		EJECT
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

		NAVIGATING_DOWN_LEFT,
		NAVIGATING_UP_LEFT,
		NAVIGATING_DOWN_RIGHT,
		NAVIGATING_UP_RIGHT,
		/**
		 * Robot is lowering intake to floor level and intaking coral gamepiece.
		 */
		LOWERING,
		/**
		 * Robot is intaking a coral gamepiece.
		 */
		INTAKING,
		EJECTING_FORWARD,
		EJECTING_BACKWARD,
		EXITING_LVL3,
		EXITING_LVL2,
		GO_TO_LVL2,
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

	public boolean LVL3Processes() {
		if (intake.hasCoral()) {
			if (
				arm.isArmAtPosition(Constants.ArmPositions.LVL3, Rotations.of(0.025)) &&
				arm.isWristAtPosition(Constants.WristPositions.LVL3, Rotations.of(0.025))
			) {
				System.out.println("Arm is in position");
				currentSuperState = SystemState.EJECTING_FORWARD;
				return false;
			}
		} else {
			if (
				arm.isArmAtPosition(Constants.ArmPositions.FLOOR, Rotations.of(0.03)) &&
				arm.isWristAtPosition(Constants.WristPositions.STOW, Rotations.of(0.01))
			) {
				targetSuperState = TargetState.DEFAULT;
				return false;
			}
			currentSuperState = SystemState.EXITING_LVL3;
			return false;
		}
		return true;
	}

	public boolean LVL2Processes() {
		if (intake.hasCoral()) {
			if (
				arm.isArmAtPosition(Constants.ArmPositions.LVL2, Rotations.of(0.025)) &&
				arm.isWristAtPosition(Constants.WristPositions.LVL2, Rotations.of(0.025))
			) {
				System.out.println("Arm is in position");
				currentSuperState = SystemState.EJECTING_BACKWARD;
				return false;
			}
		} else {
			if ((drivetrain.nearestPose != null) && (drivetrain.getState().Pose != null)) {
				routeComplete = drivetrain.nearestPose.getTranslation().getDistance(
					drivetrain.getState().Pose.getTranslation()
			) > 0.25;
				targetSuperState = TargetState.DEFAULT;
				return false;
			}
			currentSuperState = SystemState.EXITING_LVL2;
			return false;
		}
		return true;
	}

	public Boolean routeComplete = false;
	private void handleStateTransitions() {

		previousSuperState = currentSuperState;

		// if ((drivetrain.nearestPose != null) && (drivetrain.getState().Pose != null)) {
		// 	routeComplete = drivetrain.nearestPose.getTranslation().getDistance(
		// 		drivetrain.getState().Pose.getTranslation()
		// 	) < 0.025;
		// }

		if (drivetrain.navigateCommand != null) {routeComplete = drivetrain.navigateCommand.isFinished();} else {routeComplete = false;};
		
		switch (targetSuperState) {
			case STOPPED:
				break;
			case DEFAULT:
				currentSuperState = SystemState.HOME;
				break;
			case LOWER:
				currentSuperState = SystemState.LOWERING;
				break;
			case INTAKE:
				currentSuperState = SystemState.INTAKING;
				break;
			case EJECT:
				currentSuperState = SystemState.EJECTING_FORWARD;
				break;
			case SCORE_DOWN_RIGHT:
				currentSuperState = SystemState.SCORING_DOWN_RIGHT;
				break;
			case SCORE_DOWN_LEFT:
				System.out.println("At Position: " + (arm.isArmAtPosition(Constants.ArmPositions.LVL2, Rotations.of(0.025)) && arm.isWristAtPosition(Constants.WristPositions.LVL2, Rotations.of(0.025))));
				if (
					!(arm.isArmAtPosition(Constants.ArmPositions.LVL2, Rotations.of(0.025)) &&
					arm.isWristAtPosition(Constants.WristPositions.LVL2, Rotations.of(0.025)))
				) {
					currentSuperState = SystemState.GO_TO_LVL2;
				} else if (!routeComplete) {
					currentSuperState = SystemState.NAVIGATING_DOWN_LEFT;
				} else if (LVL2Processes()) {currentSuperState = SystemState.SCORING_UP_LEFT;}
				break;
			case SCORE_UP_RIGHT:
				currentSuperState = SystemState.SCORING_UP_RIGHT;
				break;
			case SCORE_UP_LEFT:
				currentSuperState = SystemState.SCORING_UP_LEFT;
				break;
			case NAVIGATE_DOWN_RIGHT:
				if (
					!(arm.isArmAtPosition(Constants.ArmPositions.LVL2, Rotations.of(0.025)) &&
					arm.isWristAtPosition(Constants.WristPositions.LVL2, Rotations.of(0.025)))
				) {
					currentSuperState = SystemState.GO_TO_LVL2;
				} else if (!routeComplete) {
					currentSuperState = SystemState.NAVIGATING_DOWN_RIGHT;
				} else if (LVL2Processes()) {currentSuperState = SystemState.SCORING_UP_RIGHT;}
				break;
			case NAVIGATE_DOWN_LEFT:
				if (
					!(arm.isArmAtPosition(Constants.ArmPositions.LVL2, Rotations.of(0.025)) &&
					arm.isWristAtPosition(Constants.WristPositions.LVL2, Rotations.of(0.025)))
				) {
					currentSuperState = SystemState.GO_TO_LVL2;
				} else if (!routeComplete) {
					currentSuperState = SystemState.NAVIGATING_DOWN_LEFT;
				} else if (LVL2Processes()) {currentSuperState = SystemState.SCORING_UP_LEFT;}
				break;
			case NAVIGATE_UP_RIGHT:
				if (!routeComplete) {
					currentSuperState = SystemState.NAVIGATING_UP_RIGHT;
				} else if (LVL3Processes()) {currentSuperState = SystemState.SCORING_UP_RIGHT;}
				break;
			case NAVIGATE_UP_LEFT:
				if (!routeComplete) {
					currentSuperState = SystemState.NAVIGATING_UP_LEFT;
				} else if (LVL3Processes()) {currentSuperState = SystemState.SCORING_UP_LEFT;}
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
			case EXITING_LVL2:
				drivetrain.targetState = DriveTrain.TargetState.TELEOP_DRIVE;
				intake.targetState = Intake.TargetState.DEFAULT;
				arm.targetState = Arm.TargetState.SCORE_LVL2;
				break;
			case HOME:
				drivetrain.targetState = DriveTrain.TargetState.TELEOP_DRIVE;
				intake.targetState = Intake.TargetState.DEFAULT;
				arm.targetState = Arm.TargetState.DEFAULT;
				break;
			case LOWERING:
				intake.targetState = Intake.TargetState.DEFAULT;
				arm.targetState = Arm.TargetState.INTAKE;
				break;
			case INTAKING:
				intake.targetState = Intake.TargetState.COLLECT;
				arm.targetState = Arm.TargetState.INTAKE;
				break;
			case EJECTING_FORWARD:
				intake.targetState = Intake.TargetState.EJECT_FORWARD;
				break;
			case EJECTING_BACKWARD:
				intake.targetState = Intake.TargetState.EJECT_BACKWARD;
				break;
			case SCORING_UP_LEFT: 
				arm.targetState = Arm.TargetState.SCORE_LVL3;
				drivetrain.targetState = DriveTrain.TargetState.STOP;
				break;
			case SCORING_UP_RIGHT:
				arm.targetState = Arm.TargetState.SCORE_LVL3;
				drivetrain.targetState = DriveTrain.TargetState.STOP;
				break;
			case SCORING_DOWN_LEFT:
				arm.targetState = Arm.TargetState.SCORE_LVL2;
				break;
			case SCORING_DOWN_RIGHT:
				arm.targetState = Arm.TargetState.SCORE_LVL2;
				break;
			case NAVIGATING_UP_LEFT:
				drivetrain.targetState = DriveTrain.TargetState.NAVIGATE_UP_LEFT;
				break;
			case NAVIGATING_UP_RIGHT:
				drivetrain.targetState = DriveTrain.TargetState.NAVIGATE_UP_RIGHT;
				break;
			case NAVIGATING_DOWN_LEFT:
				drivetrain.targetState = DriveTrain.TargetState.NAVIGATE_DOWN_LEFT;
				break;
			case NAVIGATING_DOWN_RIGHT:
				drivetrain.targetState = DriveTrain.TargetState.NAVIGATE_DOWN_RIGHT;
				break;
			case EXITING_LVL3:
				drivetrain.targetState = DriveTrain.TargetState.TELEOP_DRIVE;
				intake.targetState = Intake.TargetState.EJECT_FORWARD;
				arm.targetState = Arm.TargetState.DEFAULT;
				break;
			case GO_TO_LVL2:
				arm.targetState = Arm.TargetState.SCORE_LVL2;
				break;
			default:
				break;
		}
	}
}