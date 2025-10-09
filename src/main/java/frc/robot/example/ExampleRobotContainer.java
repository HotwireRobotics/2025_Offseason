package frc.robot.example;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.example.ExampleDriveTrain.Navigate;

public class ExampleRobotContainer {

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(Constants.MaxSpeed);


	public final ExampleDriveTrain drivetrain = new ExampleDriveTrain(TunerConstants.DrivetrainConstants, 
													TunerConstants.FrontLeft, TunerConstants.FrontRight, 
													TunerConstants.BackLeft, TunerConstants.BackRight);

	public final ExampleArm arm = new ExampleArm();

    public final ExampleSuperstructure superstructure = new ExampleSuperstructure(this);


    private final SendableChooser<Command> autoChooser;

    public ExampleRobotContainer() {
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Selected Auto", autoChooser);
    }

    private void configureBindings() {

		Constants.driver.a().onTrue(
            superstructure.Score(ExampleDriveTrain.ScoringPosition.RIGHT_L2)
        );

        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> brake));

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        Constants.driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        Constants.driver.b().whileTrue(drivetrain.applyRequest(
            () -> point.withModuleDirection(new Rotation2d(-Constants.driver.getLeftY(), -Constants.driver.getLeftX()))));

        Constants.driver.back().and(Constants.driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        Constants.driver.back().and(Constants.driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        Constants.driver.start().and(Constants.driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        Constants.driver.start().and(Constants.driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        Constants.driver.povDown().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
