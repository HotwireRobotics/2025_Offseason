// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

public class RobotContainer {
    

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(Constants.MaxSpeed);

    

    public final DriveTrain drivetrain = TunerConstants.createDrivetrain();

    public final Superstructure superstructure = new Superstructure(drivetrain);

    private final SendableChooser<Command> autoChooser;

    // TODO
    // Non-functional; Implement later
    // Orchestra music = new Orchestra(
    // Filesystem.getDeployDirectory() + "/orchestra/output.chirp"
    // );

    public RobotContainer() {
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Selected Auto", autoChooser);
    }

    private void configureBindings() {

        // This is required for `periodic()` to function.
        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> brake));

        try {
            // Load the path we want to pathfind to and follow
            PathPlannerPath path = PathPlannerPath.fromPathFile("Score");

            Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path, Constants.constraints);

            Constants.joystick.x().onTrue(pathfindingCommand);
        } catch (Exception error) {
            System.out.println(error);
        }

        

        Constants.joystick.povRight().onTrue(CommandGenerator.goRightTwoPoleLengths(drivetrain));
        Constants.joystick.povLeft().onTrue(CommandGenerator.goLeftTwoPoleLengths(drivetrain));

        // Constants.joystick.y().onTrue(new GoToNearestTag(drivetrain));
        Constants.joystick.leftBumper().onTrue(CommandGenerator.goToNearestPole(drivetrain, Tracks.left));
        Constants.joystick.rightBumper().onTrue(CommandGenerator.goToNearestPole(drivetrain, Tracks.right));
        

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        Constants.joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        Constants.joystick.b().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-Constants.joystick.getLeftY(), -Constants.joystick.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        Constants.joystick.back().and(Constants.joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        Constants.joystick.back().and(Constants.joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        Constants.joystick.start().and(Constants.joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        Constants.joystick.start().and(Constants.joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        Constants.joystick.povDown().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // return Commands.print("No autonomous command configured");
        // return new PathPlannerAuto("New Auto");
        return autoChooser.getSelected();
    }
}