package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Tracks;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Superstructure;

public class SetTargetPose extends Command {
	public DriveTrain drivetrain;
	public Superstructure superstructure;
	public Pose2d pose;
	public Superstructure.TargetState superState;

	public SetTargetPose(Superstructure superstructure, DriveTrain drivetrain, Pose2d pose, Superstructure.TargetState targetSuperState) {
		this.drivetrain = drivetrain;
		this.superstructure = superstructure;
		this.pose = pose;
		this.superState = targetSuperState;
	}

	public void initialize() {
		superstructure.targetSuperState = superState;

		drivetrain.wantedPose = pose;

		new CommandWrapper(AutoBuilder.pathfindToPose(
			drivetrain.wantedPose, Constants.constraints
		), () -> {
			superstructure.targetSuperState = Superstructure.TargetState.DEFAULT;
		}, () -> {}).schedule();
	}

	public void execute() {

	}

	public void end(boolean interrupted) {
		
	}

	public boolean isFinished() {
		return false;
	}
}