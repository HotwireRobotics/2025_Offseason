package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants.Tracks;

public class GoToNearestPole extends Command {

	public DriveTrain drivetrain;

	public GoToNearestPole(DriveTrain drivetrain) {
		this.drivetrain = drivetrain;
	}

	public void initialize() {
		Pose2d nearestPose = Constants.nearestPolePose(drivetrain.getState().Pose, Tracks.all).get();
		Command command = AutoBuilder.pathfindToPose(nearestPose, Constants.constraints);
		System.out.println("Function Start!");
		
		command.schedule();
	}

	public void execute() {

	}

	public void end(boolean interrupted) {
		System.out.println("Function End!");
	}

	public boolean isFinished() {
		return false;
	}
}


