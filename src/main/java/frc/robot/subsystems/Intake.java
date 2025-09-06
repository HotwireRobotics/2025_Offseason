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
		HOLD,
		COLLECT,
		EJECT_FORWARD,
		EJECT_BACKWARD
	}

	public TargetState targetState = TargetState.STOPPED;

	public enum SystemState {
		/**
		 * Robot is offline, move to starting positons.
		 */
		STOPPED, 
		INTAKING_CORAL,
		HOLDING_CORAL,
		INDEXING_CORAL,
		INDEX_FORWARD,
		INDEX_BACKWARD,
		EJECTING_FORWARD,
		EJECTING_BACKWARD,
		NO_CORAL
	}

	public SystemState currentState = SystemState.STOPPED;
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

	private Distance left;
	private Distance right;
	private Distance stop;
	private Distance front;

	private Boolean is_left;
	private Boolean is_right;
	private Boolean is_stop;
	private Boolean is_front;
	public void periodic() {
		left = getMeasurement(Range.LEFT);
		right = getMeasurement(Range.RIGHT);
		stop = getMeasurement(Range.STOP);
		front = getMeasurement(Range.FRONT);

		is_left  = left.magnitude() <=  Constants.Ranges.horizontal_range.magnitude();
		is_right = right.magnitude() <= Constants.Ranges.horizontal_range.magnitude();
		is_stop  = stop.magnitude() <=  Constants.Ranges.normal_range.magnitude();
		is_front = front.magnitude() <= Constants.Ranges.normal_range.magnitude();

		handleStateTransitions(); applyStates();
	}

	private void handleStateTransitions() {
		switch (targetState) {
			case STOPPED:
				currentState = SystemState.STOPPED;
				break;
			case COLLECT:
				currentState = SystemState.INTAKING_CORAL;

				if (is_stop) {
					currentState = SystemState.HOLDING_CORAL;
				} else if (is_left && is_right) {
					currentState = SystemState.INDEXING_CORAL;
				} else {
					currentState = SystemState.INTAKING_CORAL;
				}
				break;
			case HOLD:
				if (is_front && is_stop) {
					currentState = SystemState.HOLDING_CORAL;
					break;
				}
				if (is_stop) {
					currentState = SystemState.INDEX_FORWARD;
					break;
				}
				if (is_front) {
					currentState = SystemState.INDEX_BACKWARD;
					break;
				}
				currentState = SystemState.NO_CORAL;
				break;
			case EJECT_FORWARD:
				if (!(is_front || is_stop)) {
					currentState = SystemState.EJECTING_FORWARD; 
				} else {
					currentState = SystemState.NO_CORAL;
				}
				break;
			case EJECT_BACKWARD:
				if (!(is_front || is_stop)) {
					currentState = SystemState.EJECTING_BACKWARD; 
				} else {
					currentState = SystemState.NO_CORAL;
				}
				break;
			default:
				currentState = SystemState.STOPPED;
				break;
		}
	}

	private void applyStates() {

		switch (currentState) {
			case STOPPED:
				// Todo: Move to starting position.
				break;
			case INTAKING_CORAL:
				setLeftIntake(Constants.IntakeSpeeds.eject);
				setRightIntake(Constants.IntakeSpeeds.eject);
				setRollers(0.4);

				break;
			case INDEXING_CORAL:
				setLeftIntake(Constants.IntakeSpeeds.eject);
				setRightIntake(-Constants.IntakeSpeeds.eject);
				setRollers(0.4);

				break;
			case HOLDING_CORAL: case NO_CORAL:
				setLeftIntake(0);
				setRightIntake(0);
				setRollers(0);

				break;
			case INDEX_FORWARD:
				setLeftIntake(-Constants.IntakeSpeeds.index);
				setRightIntake(-Constants.IntakeSpeeds.index);
				setRollers(0);

				break;
			case INDEX_BACKWARD:
				setLeftIntake(Constants.IntakeSpeeds.index);
				setRightIntake(Constants.IntakeSpeeds.index);
				setRollers(0);

				break;
			case EJECTING_BACKWARD:
				setLeftIntake(Constants.IntakeSpeeds.eject);
				setRightIntake(Constants.IntakeSpeeds.eject);
				setRollers(0);

				break;
			case EJECTING_FORWARD:
				setLeftIntake(Constants.IntakeSpeeds.eject);
				setRightIntake(Constants.IntakeSpeeds.eject);
				setRollers(0);
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
	public Distance getMeasurement(Range range) {
		CANrange range_obj;
		switch (range) {
			case RIGHT: range_obj = r_right; break;
			case LEFT: range_obj = r_left; break;
			case FRONT: range_obj = r_front; break;
			case STOP: range_obj = r_stop; break;
			default: return Meters.of(-1); //! Error
		}
		return range_obj.getDistance().getValue();
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
