package frc.robotnew.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robotnew.Constants;

/**
 * Example subsystem class.
 */
public class Arm extends SubsystemBase {

	//* -------- Hardware --------

	/**
	 * Arm base motor.
	 */
	TalonFX m_base;
	/**
	 * Wrist motor.
	 */
	TalonFX m_wrist;

	//* ----- Initialization -----

	/**
	 * Constructor for the subsystem.
	 */
	public Arm() {
		m_base = new TalonFX(Constants.MotorIDs.arm_base_id);
		m_wrist = new TalonFX(Constants.MotorIDs.arm_wrist_id);
	}

	Command command;

	//* ----- Periodic -----

	/**
	 * Periodic method for the subsystem.
	 */
	@Override
	public void periodic() {
		
	}

	//* ----- State Management -----
	
	/**
	 * Subsystem state options.
	 */
	public enum State {
		STOPPED,
		TRANSIT,

		FLOOR,
		L2,L3,
		AGEL2,
		AGEL3,
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

	//* ----- Private Methods -----

	/**
	 * Sets the target position of <code>m_base</code>.
	 * 
	 * @param angle
	 */
	private void setArmPose(Angle angle) {
		m_base.setControl(Constants.Requests.MOTIONMAGIC.withPosition(angle));
	}

	/**
	 * Sets the target position of <code>m_wrist</code>.
	 * 
	 * @param angle
	 */
	private void setWristPose(Angle angle) {
		m_wrist.setControl(Constants.Requests.MOTIONMAGIC.withPosition(angle));
	}

	private void killArm() {
		m_base.setControl(Constants.Requests.ZERO);
	}

	private void killWrist() {
		m_wrist.setControl(Constants.Requests.ZERO);
	}

	/**
	 * Checks if <code>m_base</code> is at the target position.
	 * 
	 * @param target
	 * @param tolerance
	 * @return true if the arm is at the target position within the given tolerance.
	 */
	private boolean isArmAtPose(Angle target, Angle tolerance) {
		return Math.abs(m_base.getPosition().getValueAsDouble() - target.in(Rotations)) <= tolerance.in(Rotations);
	}

	/**
	 * Checks if <code>m_wrist</code> is at the target position.
	 * 
	 * @param target
	 * @param tolerance
	 * @return true if the wrist is at the target position within the given tolerance.
	 */
	private boolean isWristAtPose(Angle target, Angle tolerance) {
		return Math.abs(m_wrist.getPosition().getValueAsDouble() - target.in(Rotations)) <= tolerance.in(Rotations);
	}

	//* ------- Commands -------

	/**
	 * Brings the arm from its' current position to a target position.
	 * 
	 * @param armTarget The target position for the arm base.
	 * @param wristTarget The target position for the wrist.
	 * 
	 * @param endState The state to set the subsystem to when the command ends.
	 */
	public class ArmToPose extends Command {

		// Target positions
		Angle wristTarget;
		Angle armTarget;

		// End state
		State endState;

		public ArmToPose(Angle armTarget, Angle wristTarget, State endState) {
			super();
			this.armTarget = armTarget;
			this.wristTarget = wristTarget;

			this.endState = endState;

			addRequirements(Arm.this);
		}

		/**
		 * Called when the command is scheduled.
		 */
		@Override
		public void initialize() {
			setSysState(State.TRANSIT);
			
			setArmPose(armTarget);
			setWristPose(wristTarget);
		}

		/**
		 * Called during execution of the command.
		 */
		@Override
		public void execute() {
			setSysState(State.TRANSIT);
		}

		/**
		 * Returns true when the command should end.
		 */
		@Override
		public boolean isFinished() {
			return (
				isArmAtPose(armTarget, Rotations.of(0.01)) &&
				isWristAtPose(wristTarget, Rotations.of(0.01))
			);
		}

		/**
		 * Called when the command ends.
		 */
		@Override
		public void end(boolean interrupted) {
			setSysState(State.FLOOR);
		}
	}

	/**
	 * Moves the arm to the intake position.
	 */
	public class ArmToIntake extends ArmToPose {
		public ArmToIntake() {
			super(Constants.ArmPositions.FLOOR, Constants.WristPositions.INTAKE, State.FLOOR);
		}

		@Override
		public void end(boolean interrupted) {
			super.end(interrupted);
			killArm();
		}
	}

	/**
	 * Moves the arm to the floor position.
	 */
	public class ArmToFloor extends ArmToPose {
		public ArmToFloor() {
			super(Constants.ArmPositions.FLOOR, Constants.WristPositions.STOW, State.FLOOR);
		}

		@Override
		public void end(boolean interrupted) {
			super.end(interrupted);
			killArm();
		}
	}

	/**
	 * Moves the arm to the L2 position.
	 */
	public class ArmToL2 extends ArmToPose {
		public ArmToL2() {
			super(Constants.ArmPositions.LVL2, Constants.WristPositions.LVL2, State.L2);
		}
	}

	/**
	 * Moves the arm to the L3 position.
	 */
	public class ArmToL3 extends ArmToPose {
		public ArmToL3() {
			super(Constants.ArmPositions.LVL3, Constants.WristPositions.LVL3, State.L3);
		}
	}

	/**
	 * Moves the arm to the Algae L3 position.
	 */
	public class ArmAGEL3 extends ArmToPose {
		public ArmAGEL3() {
			super(Constants.ArmPositions.TAKE_ALGAE_L3, Constants.WristPositions.TAKE_L2, State.L3);
		}
	}

	/**
	 * Moves the arm to the Algae L2 position.
	 */
	public class ArmAGEL2 extends ArmToPose {
		public ArmAGEL2() {
			super(Constants.ArmPositions.TAKE_ALGAE_L2, Constants.WristPositions.TAKE_ALGAE_L3, State.L3);
		}
	}
}