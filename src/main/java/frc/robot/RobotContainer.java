// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.Tracks;
import frc.robot.commands.CommandGenerator;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SetTargetPose;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.TargetState;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.drivetrain.GyroIO;
import frc.robot.subsystems.drivetrain.GyroIOPigeon2;
import frc.robot.subsystems.drivetrain.ModuleIO;
import frc.robot.subsystems.drivetrain.ModuleIOSim;
import frc.robot.subsystems.drivetrain.ModuleIOTalonFX;
import frc.robot.subsystems.drivetrain.SwerveDriveTrain;
import frc.robot.subsystems.intake.Intake;
import frc.robotnew.subsystems.DriveTrain;

import java.lang.annotation.Target;
import java.nio.file.Path;
import java.nio.file.Paths;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(Constants.MaxSpeed);

    /**
	 * <strong>Drivetrain Subsystem</strong>
	 */
    public final Drive drivetrain;
    /**
	 * <strong>Arm Subsystem</strong>
	 */
    public final Arm arm;
    /**
	 * <strong>Intake Subsystem</strong>
	 */
    public final Intake intake;
    /**
	 * <strong>State Superstructure</strong>
	 */
    public final Superstructure superstructure;

    private final LoggedDashboardChooser<Command> autoChooser;

    // TODO

    public RobotContainer() {
        switch (Constants.currentMode) {
        case REAL:
            // Real robot, instantiate hardware IO implementations
            drivetrain =
                new Drive(
                    new GyroIOPigeon2(),
                    new ModuleIOTalonFX(TunerConstants.FrontLeft),
                    new ModuleIOTalonFX(TunerConstants.FrontRight),
                    new ModuleIOTalonFX(TunerConstants.BackLeft),
                    new ModuleIOTalonFX(TunerConstants.BackRight));
            intake =         new Intake();
            arm =            new Arm();
            superstructure = new Superstructure(this);
            break;

        case SIM:
            // Sim robot, instantiate physics sim IO implementations
            drivetrain =
                new Drive(
                    new GyroIO() {},
                    new ModuleIOSim(TunerConstants.FrontLeft),
                    new ModuleIOSim(TunerConstants.FrontRight),
                    new ModuleIOSim(TunerConstants.BackLeft),
                    new ModuleIOSim(TunerConstants.BackRight));

            intake =         new Intake() {};
            arm =            new Arm() {};
            superstructure = new Superstructure(this) {};
            break;

        default:
            // Replayed robot, disable IO implementations
            drivetrain =
                new Drive(
                    new GyroIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {});
            intake =         new Intake() {};
            arm =            new Arm() {};
            superstructure = new Superstructure(this) {};
            break;
        }

        SmartDashboard.putString("Auto Step", "None");
        NamedCommands.registerCommand("ExitStart", new InstantCommand(() -> {
            SmartDashboard.putString("Auto Step", "ExitStart");
            superstructure.targetState = Superstructure.TargetState.EXIT_STARTING_POSE;
        }));
        class ScoreL3 extends Command {
            @Override
            public void initialize() {
                SmartDashboard.putString("Auto Step", "GoToL3");
                superstructure.targetState = Superstructure.TargetState.GO_TO_LVL3;
            }

            @Override
            public void execute() {
                if (
                    arm.isArmAtPosition(Constants.ArmPositions.LVL3, Rotations.of(0.025)) &&
                    arm.isWristAtPosition(Constants.WristPositions.LVL3, Rotations.of(0.025))
                ) {
                    intake.targetState = Intake.TargetState.EJECT_FORWARD;
                    arm.targetState = Arm.TargetState.DEFAULT;
                }
            }

            @Override
            public void end(boolean interrupted) {
                superstructure.targetState = Superstructure.TargetState.AUTONOMOUS;
            }

            @Override
            public boolean isFinished() {
                return !intake.hasCoral();
            }
        }
        NamedCommands.registerCommand("GoToL3", new ScoreL3());
        NamedCommands.registerCommand("EjectCoral", new InstantCommand(() -> {
            SmartDashboard.putString("Auto Step", "EjectCoral");
            superstructure.targetState = Superstructure.TargetState.EJECT;
        }));
        NamedCommands.registerCommand("ToDefault", new InstantCommand(() -> {
            SmartDashboard.putString("Auto Step", "ToDefault");
            superstructure.targetState = Superstructure.TargetState.DEFAULT;
        }));
        NamedCommands.registerCommand("ToAlgaeL2", new InstantCommand(() -> {
            SmartDashboard.putString("Auto Step", "ToAlgaeL2");
            superstructure.targetState = Superstructure.TargetState.ALGAE_L2_AUTO;
        }));
        
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        // Set up SysId routines
        autoChooser.addOption(
            "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drivetrain));
        autoChooser.addOption(
            "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drivetrain));
        autoChooser.addOption(
            "Drive SysId (Quasistatic Forward)",
            drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
            "Drive SysId (Quasistatic Reverse)",
            drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
            "Drive SysId (Dynamic Forward)", drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
            "Drive SysId (Dynamic Reverse)", drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
            
        configureBindings();
    }

    // Cycle between positions
    Angle[] positions = {
        Constants.WristPositions.INTAKE, 
        Constants.WristPositions.STOW,
    };
    int position_index = 0;

    private void configureBindings() {
        // music.addInstrument(arm.getBaseMotor());
        // music.play(); // TODO Make orchestra function.

        // Default command, normal field-relative drive
        drivetrain.setDefaultCommand(drivetrain.driveCommand);
        
        //#####################################INTAKE#########################################

        // Lower the intake.
        Constants.operator.x().whileTrue(
            new InstantCommand(() -> {
                System.out.println("Lower");
                superstructure.targetState = Superstructure.TargetState.LOWER;
            })
        ).onFalse(
            new InstantCommand(() -> {
                System.out.println("Default");
                superstructure.targetState = Superstructure.TargetState.DEFAULT;
            })
        );

        // Run intake devices.
        Constants.operator.y().whileTrue(
            new InstantCommand(() -> {
                System.out.println("Intake");
                superstructure.targetState = Superstructure.TargetState.INTAKE;
            })
        ).onFalse(
            new InstantCommand(() -> {
                System.out.println("Default");
                superstructure.targetState = Superstructure.TargetState.DEFAULT;
            })
        );

        //####################################ABORT BUTTONS###################################

        // Run arm up.
        Constants.driver.povUp().whileTrue(
            new InstantCommand(() -> {
                System.out.println("Exit Start");
                superstructure.targetState = Superstructure.TargetState.EXIT_STARTING_POSE;
            })
        ).onFalse(
            new InstantCommand(() -> {
                System.out.println("Default");
                superstructure.targetState = Superstructure.TargetState.DEFAULT;
            })
        );
        Constants.operator.povUp().whileTrue(
            new InstantCommand(() -> {
                System.out.println("Exit Start");
                superstructure.targetState = Superstructure.TargetState.EXIT_STARTING_POSE;
            })
        ).onFalse(
            new InstantCommand(() -> {
                System.out.println("Default");
                superstructure.targetState = Superstructure.TargetState.DEFAULT;
            })
        );

        // Eject.
        Constants.driver.povRight().whileTrue(
            new InstantCommand(() -> {
                System.out.println("Eject");
                superstructure.targetState = Superstructure.TargetState.EJECT;
            })
        ).onFalse(
            new InstantCommand(() -> {
                System.out.println("Default");
                superstructure.targetState = Superstructure.TargetState.DEFAULT;
            })
        );
        Constants.operator.povRight().whileTrue(
            new InstantCommand(() -> {
                System.out.println("Eject");
                superstructure.targetState = Superstructure.TargetState.EJECT;
            })
        ).onFalse(
            new InstantCommand(() -> {
                System.out.println("Default");
                superstructure.targetState = Superstructure.TargetState.DEFAULT;
            })
        );

        //####################################################################################

        // Scoring Positions
        Constants.operator.rightBumper().onTrue(
            new InstantCommand(() -> {
                System.out.println("Up-Right");
                superstructure.targetState = Superstructure.TargetState.NAVIGATE_UP_RIGHT;
            })
        );

        Constants.operator.leftBumper().onTrue(
            new InstantCommand(() -> {
                System.out.println("Up-Left");
                superstructure.targetState = Superstructure.TargetState.NAVIGATE_UP_LEFT;
            })
        );

        //####################################################################################
        
        // Remove Algae
        Constants.operator.povDown().onTrue(
            new InstantCommand(() -> {
                superstructure.targetState = Superstructure.TargetState.NAVIGATE_ALGAE;
            })
        );
        
        //####################################################################################

        Constants.operator.rightTrigger().onTrue(
            new InstantCommand(() -> {
                System.out.println("Down-Right");
                superstructure.targetState = Superstructure.TargetState.NAVIGATE_DOWN_RIGHT;
            })
        );

        Constants.operator.leftTrigger().onTrue(
            new InstantCommand(() -> {
                System.out.println("Down-Left");
                superstructure.targetState = Superstructure.TargetState.NAVIGATE_DOWN_LEFT;
            })
        );

        //####################################################################################

        // Abort function.
        Constants.operator.back().onTrue(
            new InstantCommand(() -> {
                System.out.println("Abort");
                superstructure.targetState = Superstructure.TargetState.DEFAULT;
            })
        );
        Constants.driver.back().onTrue(
            new InstantCommand(() -> {
                System.out.println("Abort");
                superstructure.targetState = Superstructure.TargetState.DEFAULT;
            })
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(Commands.runOnce(drivetrain::stop, drivetrain).ignoringDisable(true));

        // Constants.driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // Constants.driver.b().whileTrue(drivetrain.applyRequest(
        //         () -> point.withModuleDirection(new Rotation2d(-Constants.driver.getLeftY(), -Constants.driver.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y. Note that each routine should be run exactly once in a single log.
        // Constants.driver.back().and(Constants.driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // Constants.driver.back().and(Constants.driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // Constants.driver.start().and(Constants.driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // Constants.driver.start().and(Constants.driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on down POV press.
        // Constants.driver.povDown().onTrue(drivetrain.runOnce(drivetrain::));

        // drivetrain.registerTelemetry(logger::telemeterize);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}