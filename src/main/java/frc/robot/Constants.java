package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.units.*;
import edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

public class Constants {

	public static final AprilTagFieldLayout taglayout = AprilTagFields.k2025ReefscapeWelded.loadAprilTagLayoutField();

	public static final Set<Integer> redtags = IntStream.rangeClosed(6, 11).boxed()
			.collect(Collectors.toUnmodifiableSet());

	public static final Set<Integer> bluetags = IntStream.rangeClosed(17, 22).boxed()
			.collect(Collectors.toUnmodifiableSet());

	public static final PathConstraints constraints = new PathConstraints(3.0, 4.0, Units.degreesToRadians(540),
			Units.degreesToRadians(720));

	public static final Distance poleOffset = Inches.of(6.5);

	public class Dimensions {
		public static Mass mass = Kilograms.of(35.153); // Kilograms

		// Bumpers
		public static Distance bumperWidth = Meters.of(0.864);
		public static Distance bumperLength = Meters.of(0.864);
	}

	public static Optional<Pose2d> nearestTagPose(Pose2d robotPose) {

		Set<Integer> tags = (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? redtags : bluetags;

		List<Pose2d> poses = taglayout.getTags().stream().filter(tag -> (tags.contains(tag.ID)))
				.map(tag -> (tag.pose.toPose2d())).toList();

		if (poses.isEmpty()) {
			return Optional.empty();
		}

		Pose2d pose = robotPose.nearest(poses);
		Transform2d offset = new Transform2d(
				new Translation2d(Dimensions.bumperLength.magnitude() / 2, new Rotation2d()), Rotation2d.k180deg);
		pose = pose.plus(offset);

		return Optional.of(pose);
	}

	public enum Tracks {
		right, left, all
	}

	public static Optional<Pose2d> nearestPolePose(Pose2d robotPose, Tracks types) {

		Set<Integer> tags = (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? redtags : bluetags;

		List<Pose2d> poses = taglayout.getTags().stream().filter(tag -> (tags.contains(tag.ID)))
				.map(tag -> (tag.pose.toPose2d())).toList();

		List<Pose2d> lefts = new ArrayList<>();
		List<Pose2d> rights = new ArrayList<>();

		Distance zero = Meters.of(0);
		Transform2d rightOffset = new Transform2d(new Translation2d(zero, poleOffset), new Rotation2d());
		Transform2d leftOffset = new Transform2d(new Translation2d(zero, poleOffset.times(-1)), new Rotation2d());

		for (Pose2d pose : poses) {
			Pose2d left = pose.transformBy(leftOffset);
			Pose2d right = pose.transformBy(rightOffset);

			rights.add(right);
			lefts.add(left);
		}

		if (poses.isEmpty()) {
			return Optional.empty();
		}

		Pose2d pose = robotPose.nearest(lefts);
		switch (types) {
		case all:
			rights.addAll(lefts);
			pose = robotPose.nearest(rights);
			break;
		case right:
			pose = robotPose.nearest(rights);
			break;
		case left:
			pose = robotPose.nearest(lefts);
			break;
		default:
			System.out.println("Did not select a valid tracking type.");
			rights.addAll(lefts);
			pose = robotPose.nearest(rights);
		}

		Transform2d offset = new Transform2d(
				new Translation2d(Dimensions.bumperLength.magnitude() / 2, new Rotation2d()), Rotation2d.k180deg);
		pose = pose.plus(offset);

		return Optional.of(pose);
	}

}