package frc.robot.example;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Constants.Tracks;
import frc.robot.subsystems.DriveTrain;

public class ExampleDriveTrain extends DriveTrain {
	
	//* ----- Initialization -----

	/**
	 * Constructor for the subsystem.
	 */
	public ExampleDriveTrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
	) {
		super(drivetrainConstants, modules);
	}

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
		TELEOP,
		AUTO,
		PATHFINDING,
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

	/**
	 * Target scoring position for navigation.
	 */
	public enum ScoringPosition {
		RIGHT_L2,
		LEFT_L2,
		RIGHT_L3,
		LEFT_L3
	}

	//* ------- Commands -------

	/**
	 * Switch to <code>TELEOP</code> state.
	 */
	public Command TeleopDrive = new InstantCommand(
		() -> setSysState(State.TELEOP)
	);

	/**
	 * Switch to <code>AUTO</code> state.
	 */
	public Command AutoDrive = new InstantCommand(
		() -> setSysState(State.AUTO)
	);

	public class Navigate extends Command {

		private Command navigation;

		/**
		 * Navigate to a scoring position.
		 * 
		 * @param position The target scoring position.
		 */
		public Navigate(ScoringPosition position) {
			super();
			
			switch (position) {
				case RIGHT_L2:
					nearestPose = Constants.nearestBranchPose(getState().Pose, Tracks.right).get();
					nearestPose = nearestPose.rotateAround(nearestPose.getTranslation(), new Rotation2d(Radians.of(Math.PI)));
					navigation = AutoBuilder.pathfindToPose(nearestPose, Constants.constraints);
					break;
				case LEFT_L2:
					nearestPose = Constants.nearestBranchPose(getState().Pose, Tracks.left).get();
					nearestPose = nearestPose.rotateAround(nearestPose.getTranslation(), new Rotation2d(Radians.of(Math.PI)));
					navigation = AutoBuilder.pathfindToPose(nearestPose, Constants.constraints);
					break;
				case RIGHT_L3:
					nearestPose = Constants.nearestBranchPose(getState().Pose, Tracks.right).get();
					navigation = AutoBuilder.pathfindToPose(nearestPose, Constants.constraints);
					break;
				case LEFT_L3:
					nearestPose = Constants.nearestBranchPose(getState().Pose, Tracks.left).get();
					navigation = AutoBuilder.pathfindToPose(nearestPose, Constants.constraints);
					break;
				default:
					this.cancel();
			}
		
		}

		@Override
		public void initialize() {
			setSysState(State.PATHFINDING);
			navigation.schedule();
		}

		@Override
		public void execute() {
			setSysState(State.PATHFINDING);
		}

		@Override
		public void end(boolean interrupted) {
			setSysState(State.TELEOP);
		}

		@Override
		public boolean isFinished() {
			return navigation.isFinished();
		}
	}
}