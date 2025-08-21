package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.units.*;
import edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.Tracks;
import frc.robot.commands.CommandGenerator;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Superstructure;

public class Constants {

	public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

	/**
	 * <strong>Driver Controller</strong>
	 */
    public static final CommandXboxController joystick = new CommandXboxController(0);

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

	public class MotorIDs {
		public static final Integer arm_base_id = 17;
		public static final Integer arm_wrist_id = 12;

		public static final Integer right_intake_id = 14;
		public static final Integer left_intake_id = 13;

		public static final Integer rollers_id = 11;
	}

	public class CANrangeIDs {
		public static final Integer right = 0;
		public static final Integer left = 1;
		public static final Integer front = 2;
		public static final Integer stop = 3;
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

	public static Optional<Pose2d> rightTwoPoleLengths(Pose2d robotPose) {
		Transform2d offset = new Transform2d(
			new Translation2d(poleOffset.magnitude() * 2, new Rotation2d(Math.PI / 2)), 
			Rotation2d.k180deg
		);

		Pose2d offsetPose = robotPose.plus(offset);

		return Optional.of(offsetPose);
	}

	public static Optional<Pose2d> leftTwoPoleLengths(Pose2d robotPose) {
		Transform2d offset = new Transform2d(
			new Translation2d( -poleOffset.magnitude() * 2, new Rotation2d(Math.PI / 2)), 
			Rotation2d.k180deg
		);

		Pose2d offsetPose = robotPose.plus(offset);

		return Optional.of(offsetPose);
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