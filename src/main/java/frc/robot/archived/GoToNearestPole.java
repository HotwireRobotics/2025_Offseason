package frc.robot.archived;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Tracks;
import frc.robot.subsystems.DriveTrain;

public class GoToNearestPole extends Command {

	public DriveTrain drivetrain;

	public GoToNearestPole(DriveTrain drivetrain) {
		this.drivetrain = drivetrain;
	}

	@Override
	public void initialize() {
		Pose2d nearestPose = Constants.nearestBranchPose(drivetrain.getState().Pose, Tracks.all).get();
		Command command = AutoBuilder.pathfindToPose(nearestPose, Constants.constraints);
		System.out.println("Function Start!");
		
		command.schedule();
	}

	@Override
	public void execute() {}

	@Override
	public void end(boolean interrupted) {
		System.out.println("Function End!");
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}


