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
		COLLECT,
		HOLD,
		EJECT,
		INDEX
	}

	public TargetState targetState = TargetState.STOPPED;

	public enum SystemState {
		/**
		 * Robot is offline, move to starting positons.
		 */
		STOPPED, 
		INTAKING_CORAL,
		HOLDING_CORAL
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

	public void periodic() {handleStateTransitions(); applyStates();}

	private void handleStateTransitions() {

		switch (targetState) {
			case STOPPED:
				currentState = SystemState.STOPPED;
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
			default:
				break;
		}
	}

	/**
     * Get the meaurement from one of the CANranges.
     *
     * @param range Get from the <code>Range</code> enum.
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

	/**
     * Takes a factor from <strong>-1 to 1</strong> and runs 
	 * the rollers at the appropriate speed.
     *
     * @param speed Factor from -1 to 1
     */
	public void runRollers(double speed) {
		d_rollers.set(speed);
	}

	/**
     * Takes a factor from <strong>-1 to 1</strong> and runs 
	 * the right intake at the appropriate speed.
     *
     * @param speed Factor from -1 to 1
     */
	public void runRightIntake(double speed) {
		d_right_intake.set(speed);
	}

	/**
     * Takes a factor from <strong>-1 to 1</strong> and runs 
	 * the left intake at the appropriate speed.
     *
     * @param speed Factor from -1 to 1
     */
	public void runLeftIntake(double speed) {
		d_left_intake.set(speed);
	}
}
