package frc.robotnew.subsystems;

import static edu.wpi.first.units.Units.Radians;

import java.util.ArrayList;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.SwerveDriveTrain;
import frc.robotnew.Constants;
import frc.robotnew.Constants.Dimensions;
import frc.robotnew.Constants.Tracks;

public class DriveTrain extends SwerveDriveTrain {
	
	//* ----- Initialization -----

	/**
	 * Constructor for the subsystem.
	 */
	public DriveTrain(
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
		LEFT_L3,
		ALGAE
	}

	//* ------- Commands -------

	/**
	 * Switch to <code>TELEOP</code> state.
	 */
	public Command TeleopDrive = new InstantCommand(
		() -> setSysState(State.TELEOP),
		this
	);

	/**
	 * Switch to <code>AUTO</code> state.
	 */
	public Command AutoDrive = new InstantCommand(
		() -> setSysState(State.AUTO),
		this
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

			addRequirements(DriveTrain.this);
			
			switch (position) {
				case RIGHT_L2:
					nearestPose = Constants.nearestBranchPose(getState().Pose, Tracks.right).get();
					nearestPose = nearestPose.rotateAround(nearestPose.getTranslation(), new Rotation2d(Radians.of(Math.PI)));
					navigation = new Pathfind(nearestPose);
					break;
				case LEFT_L2:
					nearestPose = Constants.nearestBranchPose(getState().Pose, Tracks.left).get();
					nearestPose = nearestPose.rotateAround(nearestPose.getTranslation(), new Rotation2d(Radians.of(Math.PI)));
					navigation = new Pathfind(nearestPose);
					break;
				case RIGHT_L3:
					nearestPose = Constants.nearestBranchPose(getState().Pose, Tracks.right).get();
					navigation = new Pathfind(nearestPose);
					break;
				case LEFT_L3:
					nearestPose = Constants.nearestBranchPose(getState().Pose, Tracks.left).get();
					navigation = new Pathfind(nearestPose);
					break;
				case ALGAE:
					nearestId = Constants.nearestAlgaeId(getState().Pose);
					nearestPose = Constants.taglayout.getTags().get(nearestId - 1).pose.toPose2d();
					Rotation2d rot = ((nearestId % 2) == (DriverStation.getAlliance().get().equals(Alliance.Red)?1:0)) ? Rotation2d.kZero : Rotation2d.k180deg;
					Transform2d offset = new Transform2d(
						new Translation2d(Dimensions.bumperLength.magnitude() / 2, new Rotation2d()), rot);
					nearestPose = nearestPose.plus(offset);
					navigation = new Pathfind(nearestPose);
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

	public class Drive extends Command {

		CommandXboxController joystick;
		Command drive;

		public Drive(CommandXboxController joystick) {
			this.joystick = joystick;

			addRequirements(DriveTrain.this);

			drive = applyRequest(() -> Constants.drive.withVelocityX(-(joystick.getLeftY()) * (joystick.x().getAsBoolean() ? Constants.MaxSpeed / 2 : Constants.MaxSpeed)) 
				// Drive forward with
				// negative Y
				// (forward)
				.withVelocityY(-(joystick.getLeftX()) * (joystick.x().getAsBoolean() ? Constants.MaxSpeed / 2 : Constants.MaxSpeed)) // Drive left with negative X (left)
				.withRotationalRate(-joystick.getRightX() * Constants.MaxAngularRate) // Drive counterclockwise with negative X (left)
			);
		}

		@Override
		public void execute() {
			setSysState(State.TELEOP);
			drive.schedule();
		}
	}


	class Pathfind extends Command {
		
		Command cmd;
		
		public Pathfind(Pose2d target) {
			cmd = AutoBuilder.pathfindToPose(target, Constants.constraints);

			addRequirements(DriveTrain.this);
		}

		@Override
		public void initialize() {
			cmd.schedule();
		}

		@Override
		public boolean isFinished() {
			return cmd.isFinished();
		}

	}
}