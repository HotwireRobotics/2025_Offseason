package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Example subsystem class.
 */
public class ExampleArm extends SubsystemBase {
	
	/**
	 * Subsystem state options.
	 */
	public enum State {
		TRANSIT,

		FLOOR,
		L2,L3,
	}

	/** 
	 * Active subsystem state.
	*/
	private State state = State.FLOOR;

	/**
	 * Returns the active subsystem state.	
	 *
	 * @return <code>state</code>
	 */
	public State getState() {
		return state;
	}

	/**
	 * Sets the active subsystem state.
	 */
	public void setState(State newState) {
		state = newState;
	}

	/**
	 * Arm base motor.
	 */
	TalonFX m_base;

	public ExampleArm() {
		m_base = new TalonFX(Constants.MotorIDs.arm_base_id);
	}

	/**
	 * Brings the arm from its' current position to floor position.
	 */
	public class ArmToFloor extends Command {

		@Override
		public void initialize() {
			setState(State.TRANSIT);
		}

		@Override
		public void execute() {
			setState(State.TRANSIT);
			setArmPose(Constants.ArmPositions.FLOOR);
		}

		@Override
		public void end(boolean interrupted) {
			setState(State.FLOOR);
		}
	}

	private void setArmPose(Angle angle) {
		m_base.setControl(Constants.Requests.MOTIONMAGIC.withPosition(angle));
	}

}