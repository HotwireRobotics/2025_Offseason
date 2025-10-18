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
		TESTING,
		/**
		 * Transition to the subsequent state based on current conditions. (This is only a <strong>TARGET</strong> state.)
		 */
		DEFAULT,
		EXIT_STARTING_POSE,
		AUTONOMOUS, 
		// Move arm and intake into scoring position.
		SCORE_DOWN_LEFT,
		SCORE_UP_LEFT,
		SCORE_DOWN_RIGHT,
		SCORE_UP_RIGHT,
		// Remove algae.
		REMOVE_ALGAE,

		// Navigate to nearest branch.
		NAVIGATE_DOWN_LEFT,
		NAVIGATE_UP_LEFT,
		NAVIGATE_DOWN_RIGHT,
		NAVIGATE_UP_RIGHT,
		NAVIGATE_EXIT_LVL2,
		// Algae
		NAVIGATE_ALGAE,
		/**
		 * Lower intake to floor level.
		 */
		LOWER,
		/**
		 * Lower intake to floor level and intake coral gamepiece.
		 */
		INTAKE,
		EJECT,
		ALGAE_L2_AUTO,

		GO_TO_LVL3,
		GO_TO_LVL2
	}

	public TargetState targetState = TargetState.STOPPED;

	public enum SystemState {
		/**
		 * Robot is offline, move to starting positons.
		 */
		STOPPED,
		TESTING,
		/**
		 * Robot is in teleop; the driver is in control.
		 */
		HOME,
		EXITING_STARTING_POSE,
		AUTONOMOUS,
		SCORING_DOWN_LEFT,
		SCORING_UP_LEFT,
		SCORING_DOWN_RIGHT,
		SCORING_UP_RIGHT,
		// Remove algae.
		REMOVING_ALGAE_L2,
		REMOVING_ALGAE_L3,

		NAVIGATING_DOWN_LEFT,
		NAVIGATING_UP_LEFT,
		NAVIGATING_DOWN_RIGHT,
		NAVIGATING_UP_RIGHT,
		NAVIGATING_EXIT_LVL2,
		// Algae
		NAVIGATING_ALGAE,
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
		// EXITING_LVL2,
		GO_TO_LVL2,
		GO_TO_LVL3,
		GOING_TO_ALGAE_L2,
		AUTO_ALGAE_L2,
	}

	public SystemState systemState = SystemState.STOPPED;

	public SystemState previousSuperState;

	private final SwerveDriveTrain drivetrain;
	private final Intake intake;
	private final Arm arm;

	public enum AutonomousState {
		RAISE_ARM_L2, LOWER_ARM,
		NAVIGATING_TO_L2
	}
	public AutonomousState autonomousState = AutonomousState.LOWER_ARM;
	
	public Superstructure(RobotContainer container) {

		this.drivetrain = container.drivetrain;
		this.intake = container.intake;
		this.arm = container.arm;

	}

	public void periodic() {
		if (drivetrain.getDoGoHome()) {
			targetState = TargetState.DEFAULT;
			drivetrain.GO_HOME = !drivetrain.GO_HOME;
		}
		if (arm.IS_ARM_AT_EXIT_STARTING_POSITION) {
			targetState = TargetState.AUTONOMOUS;
		}
		handleStateTransitions(); applyStates();
	}

	public boolean LVL3Processes() {
		if (intake.hasCoral()) {
			if (
				arm.isArmAtPosition(Constants.ArmPositions.LVL3, Rotations.of(0.025)) &&
				arm.isWristAtPosition(Constants.WristPositions.LVL3, Rotations.of(0.025))
			) {
				System.out.println("Arm is in position");
				systemState = SystemState.EJECTING_FORWARD;
				return false;
			}
		} else {
			if (
				arm.isArmAtPosition(Constants.ArmPositions.FLOOR, Rotations.of(0.03)) &&
				arm.isWristAtPosition(Constants.WristPositions.STOW, Rotations.of(0.01))
			) {
				targetState = TargetState.DEFAULT;
				return false;
			}
			systemState = SystemState.EXITING_LVL3;
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
				systemState = SystemState.EJECTING_BACKWARD;
				return false;
			}
		} else {
			// if ((drivetrain.nearestPose != null) && (drivetrain.getState().Pose != null)) {
			// 	routeComplete = drivetrain.nearestPose.getTranslation().getDistance(
			// 		drivetrain.getState().Pose.getTranslation()
			// ) > 0.25;
			// 	targetSuperState = TargetState.DEFAULT;
			// 	return false;
			// }
			targetState = TargetState.NAVIGATE_EXIT_LVL2;
			return false;
		}
		return true;
	}

	public Boolean routeComplete = false;
	private void handleStateTransitions() {

		previousSuperState = systemState;

		if (drivetrain.navigateCommand != null) {routeComplete = drivetrain.navigateCommand.isFinished();} else {routeComplete = false;};

		switch (targetState) {
			case STOPPED:
				break;
			case DEFAULT:
				systemState = SystemState.HOME;
				break;
			case TESTING:
				systemState = SystemState.TESTING;
				break;
			case EXIT_STARTING_POSE:
				systemState = SystemState.EXITING_STARTING_POSE;
				break;
			case LOWER:
				systemState = SystemState.LOWERING;
				break;
			case INTAKE:
				systemState = SystemState.INTAKING;
				break;
			case EJECT:
				if (arm.getWristMotor().gt(Constants.WristPositions.LVL3.plus(Rotations.of(0.0315))))
					systemState = SystemState.EJECTING_BACKWARD;
				else {
					systemState = SystemState.EJECTING_FORWARD;
				}
				break;
			case SCORE_DOWN_RIGHT:
				systemState = SystemState.SCORING_DOWN_RIGHT;
				break;
			case SCORE_DOWN_LEFT:
				System.out.println("At Position: " + (arm.isArmAtPosition(Constants.ArmPositions.LVL2, Rotations.of(0.025)) && arm.isWristAtPosition(Constants.WristPositions.LVL2, Rotations.of(0.025))));
				if (
					!(arm.isArmAtPosition(Constants.ArmPositions.LVL2, Rotations.of(0.025)) &&
					arm.isWristAtPosition(Constants.WristPositions.LVL2, Rotations.of(0.025)))
				) {
					systemState = SystemState.GO_TO_LVL2;
				} else if (!routeComplete) {
					systemState = SystemState.NAVIGATING_DOWN_LEFT;
				} else if (LVL2Processes()) {systemState = SystemState.SCORING_UP_LEFT;}
				break;
			case SCORE_UP_RIGHT:
				systemState = SystemState.SCORING_UP_RIGHT;
				break;
			case SCORE_UP_LEFT:
				systemState = SystemState.SCORING_UP_LEFT;
				break;
			case NAVIGATE_DOWN_RIGHT:
				if (
					!(arm.isArmAtPosition(Constants.ArmPositions.LVL2, Rotations.of(0.025)) &&
					arm.isWristAtPosition(Constants.WristPositions.LVL2, Rotations.of(0.025)))
				) {
					systemState = SystemState.GO_TO_LVL2;
				} else if (!routeComplete) {
					systemState = SystemState.NAVIGATING_DOWN_RIGHT;
				} else if (LVL2Processes()) {systemState = SystemState.SCORING_UP_RIGHT;}
				break;
			case NAVIGATE_DOWN_LEFT:
				if (
					!(arm.isArmAtPosition(Constants.ArmPositions.LVL2, Rotations.of(0.025)) &&
					arm.isWristAtPosition(Constants.WristPositions.LVL2, Rotations.of(0.025)))
				) {
					systemState = SystemState.GO_TO_LVL2;
				} else if (!routeComplete) {
					systemState = SystemState.NAVIGATING_DOWN_LEFT;
				} else if (LVL2Processes()) {systemState = SystemState.SCORING_UP_LEFT;}
				break;
			case NAVIGATE_UP_RIGHT:
				if (!routeComplete && intake.hasCoral()) {
					systemState = SystemState.NAVIGATING_UP_RIGHT;
				} else if (LVL3Processes()) {systemState = SystemState.SCORING_UP_RIGHT;}
				break;
			case NAVIGATE_UP_LEFT:
				if (!routeComplete && intake.hasCoral()) {
					systemState = SystemState.NAVIGATING_UP_LEFT;
				} else if (LVL3Processes()) {systemState = SystemState.SCORING_UP_LEFT;}
				break;
			case NAVIGATE_EXIT_LVL2:
				if ((drivetrain.nearestPose != null) && (drivetrain.getState().Pose != null)) {
					SmartDashboard.putNumber("Distance from nearestPose", drivetrain.nearestPose.getTranslation().getDistance(
						drivetrain.getState().Pose.getTranslation()
					));
					systemState = SystemState.NAVIGATING_EXIT_LVL2;
				}
				break;
			case NAVIGATE_ALGAE:
				if (!routeComplete) {
					if (!drivetrain.getAlgaePose()) {
						systemState = SystemState.GOING_TO_ALGAE_L2;
						if (
							arm.isArmAtPosition(Constants.ArmPositions.TAKE_ALGAE_L2, Rotations.of(0.0025)) &&
							arm.isWristAtPosition(Constants.WristPositions.TAKE_L2, Rotations.of(0.005))
						) {
							systemState = SystemState.NAVIGATING_ALGAE;
						}
					} else {
						systemState = SystemState.NAVIGATING_ALGAE;
					}
				} else {
					targetState = TargetState.REMOVE_ALGAE;
				}
				break;
			case REMOVE_ALGAE:
				drivetrain.targetState = SwerveDriveTrain.TargetState.TELEOP_DRIVE;
				if (drivetrain.getAlgaePose()) {
					systemState = SystemState.REMOVING_ALGAE_L3;
				} else {
					systemState = SystemState.REMOVING_ALGAE_L2;
				}
				break;
			case ALGAE_L2_AUTO:
				systemState = SystemState.AUTO_ALGAE_L2;
				break;
			case GO_TO_LVL3:
				systemState = SystemState.GO_TO_LVL3;
				break;
			case GO_TO_LVL2:
				systemState = SystemState.GO_TO_LVL2;
				break;
			case AUTONOMOUS:
				systemState = SystemState.AUTONOMOUS;
				break;
			default:
				systemState = SystemState.STOPPED;
				break;
		}
	}

	private void applyStates() {

		switch (systemState) {
			case STOPPED:
				drivetrain.targetState = SwerveDriveTrain.TargetState.IDLE;
				intake.targetState = Intake.TargetState.STOPPED;
				arm.targetState = Arm.TargetState.STOP;
				break;
			case TESTING:
				drivetrain.targetState = SwerveDriveTrain.TargetState.IDLE;
				intake.targetState = Intake.TargetState.DEFAULT;
				arm.targetState = Arm.TargetState.RUNTOPOSE;
				break;
			case NAVIGATING_EXIT_LVL2:
				// drivetrain.targetState = DriveTrain.TargetState.NAVIGATE_EXIT_LVL2;
				drivetrain.targetState = SwerveDriveTrain.TargetState.TELEOP_DRIVE;
				intake.targetState = Intake.TargetState.DEFAULT;
				arm.targetState = Arm.TargetState.SCORE_LVL2;
				break;
			case HOME:
				drivetrain.targetState = SwerveDriveTrain.TargetState.TELEOP_DRIVE;
				intake.targetState = Intake.TargetState.DEFAULT;
				arm.targetState = Arm.TargetState.DEFAULT;
				break;
			case AUTONOMOUS:
				drivetrain.targetState = SwerveDriveTrain.TargetState.AUTONOMOUS;
				intake.targetState = Intake.TargetState.DEFAULT;
				arm.targetState = Arm.TargetState.DEFAULT;
				break;
			// case AUTONOMOUS:
			// 	drivetrain.targetState = DriveTrain.TargetState.AUTONOMOUS;
			// 	intake.targetState = Intake.TargetState.DEFAULT;
			// 	arm.targetState = Arm.TargetState.DEFAULT;
			// 	switch (autonomousState) {
			// 		case RAISE_ARM_L2:
			// 			intake.targetState = Intake.TargetState.DEFAULT;
			// 			arm.targetState = Arm.TargetState.SCORE_LVL2;
			// 			if (arm.isArmAtPosition(Constants.ArmPositions.LVL3, Rotations.of(0.025)) &&
			// 				arm.isWristAtPosition(Constants.WristPositions.LVL3, Rotations.of(0.025))
			// 			) {
			// 				autonomousState = AutonomousState.NAVIGATING_TO_L2;
			// 			}
			// 			break;
			// 		case NAVIGATING_TO_L2:
			// 			intake.targetState = Intake.TargetState.DEFAULT;
			// 			arm.targetState = Arm.TargetState.SCORE_LVL2;
			// 			break;
			// 		default:
			// 			break;
			// 	}
			// 	break;
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
				drivetrain.targetState = SwerveDriveTrain.TargetState.STOP;
				break;
			case SCORING_UP_RIGHT:
				arm.targetState = Arm.TargetState.SCORE_LVL3;
				drivetrain.targetState = SwerveDriveTrain.TargetState.STOP;
				break;
			case REMOVING_ALGAE_L2:
				intake.targetState = Intake.TargetState.TAKE_ALGAE_L2;
				arm.targetState = Arm.TargetState.TAKE_ALGAE_L2;
				break;
			case REMOVING_ALGAE_L3:
				intake.targetState = Intake.TargetState.TAKE_ALGAE_L3;
						
				if (intake.getMeasurement(Intake.Range.FRONT)) {
					arm.targetState = Arm.TargetState.DEFAULT;
				} else if (
					arm.isArmAtPosition(Constants.ArmPositions.TAKE_ALGAE_L3, Rotations.of(0.015)) &&
					arm.isWristAtPosition(Constants.WristPositions.TAKE_ALGAE_L3, Rotations.of(0.015))
				) {
					arm.targetState = Arm.TargetState.REMOVE_ALGAE_L3;
				}
				
				if (arm.currentState.equals(Arm.SystemState.HOME)) {
					targetState = TargetState.DEFAULT;
				}
				break;
			case SCORING_DOWN_LEFT:
				arm.targetState = Arm.TargetState.SCORE_LVL2;
				break;
			case SCORING_DOWN_RIGHT:
				arm.targetState = Arm.TargetState.SCORE_LVL2;
				break;
			case NAVIGATING_UP_LEFT:
				drivetrain.targetState = SwerveDriveTrain.TargetState.NAVIGATE_UP_LEFT;
				break;
			case NAVIGATING_UP_RIGHT:
				drivetrain.targetState = SwerveDriveTrain.TargetState.NAVIGATE_UP_RIGHT;
				break;
			case NAVIGATING_DOWN_LEFT:
				drivetrain.targetState = SwerveDriveTrain.TargetState.NAVIGATE_DOWN_LEFT;
				break;
			case NAVIGATING_DOWN_RIGHT:
				drivetrain.targetState = SwerveDriveTrain.TargetState.NAVIGATE_DOWN_RIGHT;
				break;
			case NAVIGATING_ALGAE:
				arm.targetState = drivetrain.getAlgaePose() ? 
					Arm.TargetState.TAKE_ALGAE_L3 : Arm.TargetState.TAKE_ALGAE_L2;
				
				drivetrain.targetState = SwerveDriveTrain.TargetState.NAVIGATE_ALGAE;
				break;
			case EXITING_LVL3:
				drivetrain.targetState = SwerveDriveTrain.TargetState.TELEOP_DRIVE;
				intake.targetState = Intake.TargetState.EJECT_FORWARD;
				arm.targetState = Arm.TargetState.DEFAULT;
				break;
			case GO_TO_LVL2:
				arm.targetState = Arm.TargetState.SCORE_LVL2;
				break;
			case AUTO_ALGAE_L2:
				intake.targetState = Intake.TargetState.TAKE_ALGAE_L2;
			case GOING_TO_ALGAE_L2:
				arm.targetState = Arm.TargetState.TAKE_ALGAE_L2;
				break;
			case EXITING_STARTING_POSE:
				intake.targetState = Intake.TargetState.DEFAULT;
				arm.targetState = Arm.TargetState.EXIT_STARTING_POSE;
				break;
			case GO_TO_LVL3:
				arm.targetState = Arm.TargetState.SCORE_LVL3;
				break;
			default:
				break;
		}
	}
}