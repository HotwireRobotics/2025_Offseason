package frc.robot.commands;

import java.util.Dictionary;
import java.util.Hashtable;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.Tracks;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Superstructure;

public class CommandGenerator {
	public static Command goToNearestPole(RobotContainer container, Tracks types) {
		return Commands.defer(() -> {
			Pose2d nearestPose = Constants.nearestPolePose(container.drivetrain.getState().Pose, types).get();
			Command command = AutoBuilder.pathfindToPose(nearestPose, Constants.constraints);

			Dictionary<Tracks, Superstructure.SystemState> tracksToState = new Hashtable<>();
			tracksToState.put(Tracks.right, Superstructure.SystemState.SCORING_CORAL_RIGHT);
			tracksToState.put(Tracks.left, Superstructure.SystemState.SCORING_CORAL_LEFT);
			
			Superstructure.SystemState state = tracksToState.get(types);

			command = new CommandWrapper(command, () -> {
				container.superstructure.targetSuperState = Superstructure.TargetState.DEFAULT;
			}, () -> {
				container.superstructure.currentSuperState = state;
			});

			return command;
		}, Set.of(container.drivetrain));
	}

	public static Command goToNearestTag(DriveTrain drivetrain) {
		return Commands.defer(() -> {
			Pose2d nearestPose = Constants.nearestTagPose(drivetrain.getState().Pose).get();
			Command command = AutoBuilder.pathfindToPose(nearestPose, Constants.constraints);

			return command;
		}, Set.of(drivetrain));
	}

	public static Command goRightTwoPoleLengths(DriveTrain drivetrain) {
		return Commands.defer(() -> {
			Pose2d pose = Constants.rightTwoPoleLengths(drivetrain.getState().Pose).get();
			Command command = AutoBuilder.pathfindToPose(pose, Constants.constraints);

			// command = new CommandWrapper(command, () -> {
			// 	drivetrain.state = DriveTrain.SystemState.at_right_pole;
			// }, () -> {});

			return command;
		}, Set.of(drivetrain));
	}

	public static Command goLeftTwoPoleLengths(DriveTrain drivetrain) {
		return Commands.defer(() -> {
			Pose2d pose = Constants.leftTwoPoleLengths(drivetrain.getState().Pose).get();
			Command command = AutoBuilder.pathfindToPose(pose, Constants.constraints);

			// command = new CommandWrapper(command, () -> {
			// 	drivetrain.state = DriveTrain.SystemState.at_left_pole;
			// }, () -> {});

			return command;
		}, Set.of(drivetrain));		
	}
}
