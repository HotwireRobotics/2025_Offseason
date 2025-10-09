package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
	
	public enum TargetState {
		/**
		 * Robot is offline, move to starting positons.
		 */
		STOPPED, 
		DEFAULT,
		COLLECT,
		EJECT_FORWARD,
		EJECT_BACKWARD,

		TAKE_ALGAE_L3,
		TAKE_ALGAE_L2
	}

	public TargetState targetState = TargetState.STOPPED;
	

	public enum SystemState {
		/**
		 * Robot is offline, move to starting positons.
		 */
		STOPPED, 
		INTAKING_CORAL,
		INDEXING_CORAL,
		INDEX_FORWARD,
		INDEX_BACKWARD,
		EJECTING_FORWARD,
		EJECTING_BACKWARD,

		NO_CORAL,
		HOLDING_CORAL,

		TAKING_ALGAE_L3,
		TAKING_ALGAE_L2
	}

	public SystemState currentState = SystemState.STOPPED;
	public SystemState getSystemState() {
		return currentState;
	}
	public SystemState previousSuperState;

	private final CANrange r_right;
	private final CANrange r_left;
	private final CANrange r_front;
	private final CANrange r_stop;

	public enum Range {
		RIGHT, LEFT, FRONT, STOP
	}

	/**
	 * <strong>Righthand wheel intake relative to robot.</strong>
	 */
	private final TalonFXS d_right_intake;
	/**
	 * <strong>Lefthand wheel intake relative to robot.</strong>
	 */
	private final TalonFXS d_left_intake;

	/**
	 * <strong>Horizontal Rollers</strong>
	 */
	private final TalonFXS d_rollers;

	public Intake() {
		r_right = new CANrange(Constants.CANrangeIDs.right);
		r_left = new CANrange(Constants.CANrangeIDs.left);
		r_front = new CANrange(Constants.CANrangeIDs.front);
		r_stop = new CANrange(Constants.CANrangeIDs.stop);

		d_right_intake = new TalonFXS(Constants.MotorIDs.right_intake_id);
		d_left_intake = new TalonFXS(Constants.MotorIDs.left_intake_id);

		d_rollers = new TalonFXS(Constants.MotorIDs.rollers_id);
	}

	private Boolean left;
	private Boolean right;
	private Boolean stop;
	private Boolean front;

	private Boolean hasCoral = false;
	public boolean hasCoral() {
		return hasCoral;
	}

	public void periodic() {
		left =  getMeasurement(Range.LEFT);
		right = getMeasurement(Range.RIGHT);
		stop =  getMeasurement(Range.STOP);
		front = getMeasurement(Range.FRONT);

		handleStateTransitions(); applyStates();

		if (front && stop) {
			hasCoral = true;
		} else {
			hasCoral = false;
		}
	}

	private void handleStateTransitions() {
		switch (targetState) {
			case STOPPED:
				currentState = SystemState.STOPPED;
				break;
			case COLLECT:
				currentState = SystemState.INTAKING_CORAL;

				if (stop) {
					currentState = SystemState.HOLDING_CORAL;
				} else if (left && right) {
					currentState = SystemState.INDEXING_CORAL;
				} else {
					currentState = SystemState.INTAKING_CORAL;
				}
				break;
			case DEFAULT:
				if (front && stop) {
					currentState = SystemState.HOLDING_CORAL;
					break;
				}
				if (stop) {
					currentState = SystemState.INDEX_FORWARD;
					break;
				}
				if (front) {
					currentState = SystemState.INDEX_BACKWARD;
					break;
				}
				currentState = SystemState.NO_CORAL;
				break;
			case EJECT_FORWARD:
				if (front || stop) {
					currentState = SystemState.EJECTING_FORWARD; 
				} else {
					currentState = SystemState.NO_CORAL;
				}
				break;
			case EJECT_BACKWARD:
				if (front || stop) {
					currentState = SystemState.EJECTING_BACKWARD; 
				} else {
					currentState = SystemState.NO_CORAL;
				}
				break;
			case TAKE_ALGAE_L2:
				currentState = SystemState.TAKING_ALGAE_L2;
				break;
			case TAKE_ALGAE_L3:
				currentState = SystemState.TAKING_ALGAE_L3;
				break;
			default:
				currentState = SystemState.STOPPED;
				break;
		}
	}

	private void applyStates() {

		switch (currentState) {
			case STOPPED:
				break;
			case INTAKING_CORAL:
				setLeftIntake(Constants.IntakeSpeeds.EJECT);
				setRightIntake(Constants.IntakeSpeeds.EJECT);
				setRollers(0.4);

				break;
			case INDEXING_CORAL:
				setLeftIntake(Constants.IntakeSpeeds.EJECT);
				setRightIntake(-Constants.IntakeSpeeds.EJECT);
				setRollers(0.4);

				break;
			case HOLDING_CORAL: case NO_CORAL:
				setLeftIntake(0);
				setRightIntake(0);
				setRollers(0);

				break;
			case INDEX_FORWARD:
				setLeftIntake(-Constants.IntakeSpeeds.INDEX);
				setRightIntake(-Constants.IntakeSpeeds.INDEX);
				setRollers(0);

				break;
			case INDEX_BACKWARD:
				setLeftIntake(Constants.IntakeSpeeds.INDEX);
				setRightIntake(Constants.IntakeSpeeds.INDEX);
				setRollers(0);

				break;
			case EJECTING_BACKWARD:
				setLeftIntake(Constants.IntakeSpeeds.EJECT);
				setRightIntake(Constants.IntakeSpeeds.EJECT);
				setRollers(0);

				break;
			case EJECTING_FORWARD:
				setLeftIntake(-Constants.IntakeSpeeds.EJECT);
				setRightIntake(-Constants.IntakeSpeeds.EJECT);
				setRollers(0);

				break;
			case TAKING_ALGAE_L2:
				setLeftIntake(-Constants.IntakeSpeeds.EJECT);
				setRightIntake(-Constants.IntakeSpeeds.EJECT);
				setRollers(-0.45);

				break;
			case TAKING_ALGAE_L3:
				setLeftIntake(Constants.IntakeSpeeds.ALGAE);
				setRightIntake(Constants.IntakeSpeeds.ALGAE);
				setRollers(0.3);

				break;
			default:
				break;
		}
	}

	/**
     * Get the meaurement from one of the CANranges.
     *
     * @param range 
	 * Get from the <code>Range</code> enum.
     */
	public Boolean getMeasurement(Range range) {
		CANrange range_obj;
		switch (range) {
			case RIGHT: range_obj = r_right; break;
			case LEFT: range_obj = r_left; break;
			case FRONT: range_obj = r_front; break;
			case STOP: range_obj = r_stop; break;
			default: return false; //! Error
		}
		return range_obj.getIsDetected().getValue();
	}

	public TalonFXS getRollers() {
		return d_rollers;
	}

	public TalonFXS getLeftIntake() {
		return d_left_intake;
	}

	public TalonFXS getRightIntake() {
		return d_right_intake;
	}

	/**
     * Takes a factor from <strong>-1 to 1</strong> and runs 
	 * the rollers at the appropriate speed.
     *
     * @param speed Factor from -1 to 1
     */
	public void setRollers(double speed) {
		d_rollers.set(speed);
	}

	/**
     * Takes a factor from <strong>-1 to 1</strong> and runs 
	 * the right intake at the appropriate speed.
     *
     * @param speed Factor from -1 to 1
     */
	public void setRightIntake(double speed) {
		d_right_intake.set(speed);
	}

	/**
     * Takes a factor from <strong>-1 to 1</strong> and runs 
	 * the left intake at the appropriate speed.
     *
     * @param speed Factor from -1 to 1
     */
	public void setLeftIntake(double speed) {
		d_left_intake.set(speed);
	}
}
