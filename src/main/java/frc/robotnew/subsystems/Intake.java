package frc.robotnew.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robotnew.Constants;
import frc.robotnew.Constants.CANrangeIDs;
import frc.robotnew.Constants.IntakeSpeeds;
import frc.robotnew.Constants.MotorIDs;

/**
 * Example subsystem class.
 */
public class Intake extends SubsystemBase {

	//* -------- Hardware --------

	/**
	 * Top intake motor.
	 */
	TalonFX m_top_intake;
	/**
	 * Right intake motor.
	 */
	TalonFX m_right_intake;
	/**
	 * Left intake motor.
	 */
	TalonFX m_left_intake;
	
	private final CANrange r_right;
	private final CANrange r_left;
	private final CANrange r_front;
	private final CANrange r_stop;

	public enum Range {
		RIGHT, LEFT, FRONT, STOP
	}

	//* ----- Initialization -----

	/**
	 * Constructor for the subsystem.
	 */
	public Intake() {
		m_right_intake = new TalonFX(Constants.MotorIDs.right_intake_id);
		m_left_intake  = new TalonFX(Constants.MotorIDs.left_intake_id);
		m_top_intake   = new TalonFX(Constants.MotorIDs.rollers_id);

		r_right = new CANrange(Constants.CANrangeIDs.right);
		r_left = new CANrange(Constants.CANrangeIDs.left);
		r_front = new CANrange(Constants.CANrangeIDs.front);
		r_stop = new CANrange(Constants.CANrangeIDs.stop);
	}

	//* ----- Periodic -----

	/**
	 * Periodic method for the subsystem.
	 */
	@Override
	public void periodic() {
		hasCoral = getRange(Range.FRONT) && getRange(Range.STOP);
		if (getSysState() == State.HOLDING) {
			double speed =
				getRange(Range.FRONT) && !getRange(Range.STOP) ?
				  Constants.IntakeSpeeds.INDEX :
				!getRange(Range.FRONT) && getRange(Range.STOP) ?
				- Constants.IntakeSpeeds.INDEX : 0;
			setLeftIntake(speed);
			setRightIntake(speed);
			setTopIntake(0.0);
		}
	}

	//* ----- State Management -----
	
	/**
	 * Subsystem state options.
	 */
	public enum State {
		STOPPED,
		EMPTY,
		HOLDING,
		REMOVING,
		INTAKING_FORWARD,
		INTAKING_REVERSE,
		EJECTING_FORWARD,
		EJECTING_REVERSE,
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

	public enum EjectDirection {
		FORWARD, REVERSE
	}

	//* ----- Private Methods -----

	/**
	 * Sets the percent voltage of <code>m_left_intake</code>.
	 * 
	 * @param speed
	 */
	private void setLeftIntake(double speed) {
		m_left_intake.set(speed);
	}

	/**
	 * Sets the percent voltage of <code>m_right_intake</code>.
	 * 
	 * @param speed
	 */
	private void setRightIntake(double speed) {
		m_right_intake.set(speed);
	}

	/**
	 * Sets the percent voltage of <code>m_top_intake</code>.
	 * 
	 * @param speed
	 */
	private void setTopIntake(double speed) {
		m_top_intake.set(speed);
	}

	/**
     * Get the meaurement from one of the CANranges.
     *
     * @param range 
	 * Get from the <code>Range</code> enum.
     */
	private Boolean getRange(Range range) {
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

	private Boolean hasCoral = false;
	private boolean hasCoral() {
		return hasCoral;
	}

	//* ------- Commands -------

	public class IntakeCoral extends Command {

		public IntakeCoral() {
			setSysState(State.INTAKING_FORWARD);

			addRequirements(Intake.this);
		}

		/**
		 * Run intake motors based on ranges.
		 */
		@Override
		public void execute() {
			setLeftIntake(
				getRange(Range.LEFT) && getRange(Range.RIGHT) ?
				- Constants.IntakeSpeeds.MAX :
				  Constants.IntakeSpeeds.MAX
			);
			setRightIntake(
				  Constants.IntakeSpeeds.MAX
			);
			setTopIntake(
				  Constants.IntakeSpeeds.MAX
			);
		}

		/**
		 * Turn off intake and set final state.
		 */
		@Override
		public void end(boolean interrupted) {
			if (interrupted) {
				setSysState(State.EMPTY);
			} else {
				setSysState(State.HOLDING);
			}
			setLeftIntake(0);
			setRightIntake(0);
			setTopIntake(0);
		}

		/**
		 * Finish when the stop range is triggered by coral.
		 */
		@Override
		public boolean isFinished() {
			return getRange(Range.STOP) && getRange(Range.FRONT);
		}
	}

	public class EjectCoral extends Command {

		private double speed;

		public EjectCoral(EjectDirection direction) {
			switch (direction) {
				case FORWARD:
					setSysState(State.EJECTING_FORWARD);
					speed = -Constants.IntakeSpeeds.MAX;
					break;
				case REVERSE:
					setSysState(State.EJECTING_REVERSE);
					speed = Constants.IntakeSpeeds.MAX;
					break;
				default:
					setSysState(State.EJECTING_FORWARD);
					speed = -Constants.IntakeSpeeds.MAX;
					break;
			}

			addRequirements(Intake.this);
		}

		/**
		 * Run intake motors in reverse to eject coral.
		 */
		@Override
		public void execute() {
			setLeftIntake(speed);
			setRightIntake(speed);
			setTopIntake(speed);
		}

		/**
		 * Turn off intake and set final state.
		 */
		@Override
		public void end(boolean interrupted) {
			setSysState(State.EMPTY);
			setLeftIntake(0);
			setRightIntake(0);
			setTopIntake(0);
		}

		/**
		 * Finish when the stop range and the front range are no longer triggered.
		 */
		@Override
		public boolean isFinished() {
			return !(getRange(Range.STOP) || getRange(Range.FRONT));
		}
	}

	public class RunIntake extends Command {

		Time time;

		double right;
		double left;
		double top;

		private final Timer timer = new Timer();

		public RunIntake(Time time, double right, double left, double top) {
			setSysState(State.REMOVING);
			this.time = time;
			this.right = right;
			this.left = left;
			this.top = top;

			addRequirements(Intake.this);
		}

		@Override
		public void initialize() {
			timer.restart();
		}

		public void execute() {
			setRightIntake(right);
			setLeftIntake(left);
			setTopIntake(top);
		}

		/**
		 * Turn off intake and set final state.
		 */
		@Override
		public void end(boolean interrupted) {
			setSysState(State.EMPTY);
			setLeftIntake(0);
			setRightIntake(0);
			setTopIntake(0);
		}

		@Override
		public boolean isFinished() {
			return timer.hasElapsed(time.in(edu.wpi.first.units.Units.Seconds));
		}
	}

	
}