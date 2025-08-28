package frc.robot.archived;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class GoToNearestTag extends Command {

	public DriveTrain drivetrain;

	public GoToNearestTag(DriveTrain drivetrain) {
		this.drivetrain = drivetrain;
	}

	@Override
	public void initialize() {
		Pose2d nearestPose = Constants.nearestTagPose(drivetrain.getState().Pose).get();
		Command command = AutoBuilder.pathfindToPose(nearestPose, Constants.constraints);

		command.schedule();
	}

	@Override
	public void execute() {}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() {
		return false;
	}
}


