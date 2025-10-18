package frc.robotnew;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drivetrain.SwerveDriveTrain;
import frc.robotnew.subsystems.Arm;
import frc.robotnew.subsystems.DriveTrain;
import frc.robotnew.subsystems.Superstructure;
import frc.robotnew.subsystems.Intake;

public class RobotContainer {

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(Constants.MaxSpeed);

	public final DriveTrain drivetrain = new DriveTrain(TunerConstants.DrivetrainConstants, 
													TunerConstants.FrontLeft, TunerConstants.FrontRight, 
													TunerConstants.BackLeft, TunerConstants.BackRight);

	public final Arm arm = new Arm();

    public final Intake intake = new Intake();

    public final Superstructure superstructure = new Superstructure(this);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureBindings();
        SmartDashboard.putString("Auto Step", "None");
        NamedCommands.registerCommand("ExitStart", superstructure.exitstart());
        NamedCommands.registerCommand("GoToL3", superstructure.score(DriveTrain.ScoringPosition.RIGHT_L3, true));
        NamedCommands.registerCommand("EjectCoral", superstructure.eject(Intake.EjectDirection.REVERSE));
        NamedCommands.registerCommand("ToDefault", superstructure.stow());
        NamedCommands.registerCommand("ToAlgaeL2", superstructure.ageL2Arm());
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Selected Auto", autoChooser);
    }

    private void configureBindings() {

        // Score on branch.
		Constants.operator.rightBumper().onTrue(
            superstructure.score(DriveTrain.ScoringPosition.RIGHT_L3, false)
        );
        Constants.operator.leftBumper().onTrue(
            superstructure.score(DriveTrain.ScoringPosition.LEFT_L3, false)
        );
        Constants.operator.rightTrigger().onTrue(
            superstructure.score(DriveTrain.ScoringPosition.RIGHT_L2, false)
        );
        Constants.operator.leftTrigger().onTrue(
            superstructure.score(DriveTrain.ScoringPosition.LEFT_L2, false)
        );

        // Algae
        Constants.operator.x().onTrue(
            superstructure.remove(DriveTrain.ScoringPosition.ALGAE)
        );

        // Abort function.
        Constants.driver.back().onTrue(
            superstructure.stow()
        );
        Constants.operator.back().onTrue(
            superstructure.stow()
        );

        // Intake coral.
        Constants.operator.y().onTrue(
            superstructure.intake()
        ).onFalse(
            superstructure.stow()
        );

        // Lower
        Constants.operator.x().onTrue(
            superstructure.lower()
        ).onFalse(
            superstructure.stow()
        );

        // drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> brake));
        drivetrain.setDefaultCommand(
            drivetrain.run(() -> {
                double lx = Constants.driver.getLeftX();
                double ly = Constants.driver.getLeftY();
                double rx = Constants.driver.getRightX();
                double ry = Constants.driver.getRightY();
                
                if (Math.abs(lx) > 0.05 || Math.abs(ly) > 0.05 
                 || Math.abs(rx) > 0.05 || Math.abs(ry) > 0.05) {
                    superstructure.drive(Constants.driver);
                } else {
                    drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake()).schedule();
                }
            })
        );


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
