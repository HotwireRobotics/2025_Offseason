package frc.robot.commands;

import java.util.Dictionary;
import java.util.Hashtable;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.Tracks;
import frc.robot.subsystems.DriveTrain;

public class CommandGenerator {
	public static Command goToNearestPole(DriveTrain drivetrain, Tracks types) {
		return Commands.defer(() -> {
			Pose2d nearestPose = Constants.nearestPolePose(drivetrain.getState().Pose, types).get();
			Command command = AutoBuilder.pathfindToPose(nearestPose, Constants.constraints);

			Dictionary<Tracks, DriveTrain.State> tracksToState = new Hashtable<>();
			tracksToState.put(Tracks.right, DriveTrain.State.at_right_pole);
			tracksToState.put(Tracks.left, DriveTrain.State.at_left_pole);
			
			DriveTrain.State state = tracksToState.get(types);

			command = new CommandWrapper(command, () -> {
				drivetrain.state = state;
			});

			return command;
		}, Set.of(drivetrain));
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

			command = new CommandWrapper(command, () -> {
				drivetrain.state = DriveTrain.State.at_right_pole;
			});

			return command;
		}, Set.of(drivetrain));
	}

	public static Command goLeftTwoPoleLengths(DriveTrain drivetrain) {
		return Commands.defer(() -> {
			Pose2d pose = Constants.leftTwoPoleLengths(drivetrain.getState().Pose).get();
			Command command = AutoBuilder.pathfindToPose(pose, Constants.constraints);

			command = new CommandWrapper(command, () -> {
				drivetrain.state = DriveTrain.State.at_left_pole;
			});

			return command;
		}, Set.of(drivetrain));		
	}
}
