package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveTrain;
import frc.robot.subsystems.Superstructure;
import frc.robotnew.Constants;
import frc.robotnew.Constants.Tracks;

public class SetTargetPose extends Command {
	public SwerveDriveTrain drivetrain;
	public Superstructure superstructure;
	public Pose2d pose;
	public Superstructure.TargetState superState;

	public SetTargetPose(Superstructure superstructure, SwerveDriveTrain drivetrain, Pose2d pose, Superstructure.TargetState targetSuperState) {
		this.drivetrain = drivetrain;
		this.superstructure = superstructure;
		this.pose = pose;
		this.superState = targetSuperState;
	}

	public void initialize() {
		superstructure.targetState = superState;

		drivetrain.wantedPose = pose;

		new CommandWrapper(AutoBuilder.pathfindToPose(
			drivetrain.wantedPose, Constants.constraints
		), () -> {
			superstructure.targetState = Superstructure.TargetState.DEFAULT;
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