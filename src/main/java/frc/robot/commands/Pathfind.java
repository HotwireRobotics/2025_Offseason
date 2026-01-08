package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.path.GoalEndState;

public class Pathfind extends Command {

    private final PathPlannerPath path;

    public Pathfind(Pose2d start, Pose2d end, PathConstraints constraints) {
		
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            start, end
        );

        path = new PathPlannerPath(
			waypoints,
			constraints,
			null,
			new GoalEndState(0.0, end.getRotation())
		);

        path.preventFlipping = true;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
