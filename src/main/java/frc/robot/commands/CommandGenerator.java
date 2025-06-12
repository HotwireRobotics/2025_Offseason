package frc.robot.commands;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants.Tracks;

public class CommandGenerator {
	public static Command goToNearestPole(CommandSwerveDrivetrain drivetrain, Tracks types) {
		return Commands.defer(() -> {
			Pose2d nearestPose = Constants.nearestPolePose(drivetrain.getState().Pose, types).get();
			Command command = AutoBuilder.pathfindToPose(nearestPose, Constants.constraints);

			return command;
		}, Set.of(drivetrain));		
	}

	public static Command goToNearestTag(CommandSwerveDrivetrain drivetrain) {
		return Commands.defer(() -> {
			Pose2d nearestPose = Constants.nearestTagPose(drivetrain.getState().Pose).get();
			Command command = AutoBuilder.pathfindToPose(nearestPose, Constants.constraints);

			return command;
		}, Set.of(drivetrain));		
	}
}
