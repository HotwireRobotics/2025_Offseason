package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class GoToNearestTag extends Command {

	public CommandSwerveDrivetrain drivetrain;

	public GoToNearestTag(CommandSwerveDrivetrain drivetrain) {
		this.drivetrain = drivetrain;
	}

	public void initialize() {
		Pose2d nearestPose = Constants.nearestTagPose(drivetrain.getState().Pose).get();
		Command command = AutoBuilder.pathfindToPose(nearestPose, Constants.constraints);

		command.schedule();
	}

	public void execute() {

	}

	public void end(boolean interrupted) {

	}

	public boolean isFinished() {
		return false;
	}
}


