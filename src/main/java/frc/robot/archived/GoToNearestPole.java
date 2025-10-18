package frc.robot.archived;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveTrain;
import frc.robotnew.Constants;
import frc.robotnew.Constants.Tracks;

public class GoToNearestPole extends Command {

	public SwerveDriveTrain drivetrain;

	public GoToNearestPole(SwerveDriveTrain drivetrain) {
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


