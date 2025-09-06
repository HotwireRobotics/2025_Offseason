// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.Tracks;
import frc.robot.commands.CommandGenerator;
import frc.robot.commands.SetTargetPose;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake;
import java.nio.file.Path;
import java.nio.file.Paths;

public class RobotContainer {

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(Constants.MaxSpeed);

    /**
	 * <strong>Drivetrain Subsystem</strong>
	 */
    public final DriveTrain drivetrain = TunerConstants.createDrivetrain();
    /**
	 * <strong>Arm Subsystem</strong>
	 */
    public final Arm arm = new Arm();
    /**
	 * <strong>Intake Subsystem</strong>
	 */
    public final Intake intake = new Intake();
    /**
	 * <strong>State Superstructure</strong>
	 */
    public final Superstructure superstructure = new Superstructure(this);

    private final SendableChooser<Command> autoChooser;

    // TODO
    // Non-functional; Implement later
    // Orchestra music = new Orchestra(
    //     Filesystem.getDeployDirectory() + "/orchestra/output.chrp"
    // );

    public RobotContainer() {
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Selected Auto", autoChooser);
    }

    private void configureBindings() {

        // music.play(); // TODO Make orchestra function.

        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> brake));

        // Constants.joystick.leftBumper().onTrue(CommandGenerator.goToNearestBranch(this, Tracks.left));
        // Constants.joystick.rightBumper().onTrue(CommandGenerator.goToNearestBranch(this, Tracks.right));

        // Left POV button navigates right to the righthand branch.
        Constants.joystick.povRight().onTrue(
            CommandGenerator.goRightTwoBranchWidths(drivetrain)
        );

        // Left POV button navigates left to the lefthand branch.
        Constants.joystick.povLeft().onTrue(
            CommandGenerator.goLeftTwoBranchWidths(drivetrain)
        );

        // Right bumper navigates to the nearest lefthand branch.
        // Constants.joystick.leftBumper().onTrue(
        //     new SetTargetPose(
        //         superstructure, drivetrain, Constants.nearestBranchPose(drivetrain.getState().Pose, Tracks.left).get(),
        //         Superstructure.TargetState.SCORE_LEFT
        //     )
        // );

        // Right bumper navigates to the nearest righthand branch.
        // Constants.joystick.rightBumper().onTrue(
        //     new SetTargetPose(
        //         superstructure, drivetrain, Constants.nearestBranchPose(drivetrain.getState().Pose, Tracks.right).get(),
        //         Superstructure.TargetState.SCORE_RIGHT
        //     )
        // );

        // // Right trigger controls percentage motor voltage.
        // Constants.joystick.rightTrigger().onTrue(
        //     new InstantCommand(() -> {
        //         arm.setBaseMotor(Constants.joystick.getRightTriggerAxis());
        //     })
        // );

        //! Temporary
        Constants.joystick.x().whileTrue(
            new InstantCommand(() -> {
                intake.targetState = Intake.TargetState.COLLECT;
            })
        ).onFalse(
            new InstantCommand(() -> {
                intake.targetState = Intake.TargetState.HOLD;
            })
        );
        

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        Constants.joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        Constants.joystick.b().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-Constants.joystick.getLeftY(), -Constants.joystick.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y. Note that each routine should be run exactly once in a single log.
        Constants.joystick.back().and(Constants.joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        Constants.joystick.back().and(Constants.joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        Constants.joystick.start().and(Constants.joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        Constants.joystick.start().and(Constants.joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on down POV press.
        Constants.joystick.povDown().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // return Commands.print("No autonomous command configured");
        // return new PathPlannerAuto("New Auto");
        return autoChooser.getSelected();
    }
}