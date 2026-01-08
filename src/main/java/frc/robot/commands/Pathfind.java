package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.path.GoalEndState;

import com.pathplanner.lib.auto.AutoBuilder;

public class Pathfind extends Command {

    private final Command follow;

    public Pathfind(Pose2d start, Pose2d end, PathConstraints constraints) {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(start, end);

        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null,
                new GoalEndState(0.0, end.getRotation())
        );

        path.preventFlipping = true;

        follow = AutoBuilder.followPath(path);
    }

    @Override
    public void initialize() {
        follow.initialize();
    }

    @Override
    public void execute() {
        follow.execute();
    }

    @Override
    public void end(boolean interrupted) {
        follow.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return follow.isFinished();
    }
}
