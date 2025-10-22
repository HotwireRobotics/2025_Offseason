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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.Tracks;
import frc.robot.commands.CommandGenerator;
import frc.robot.commands.SetTargetPose;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.TargetState;
import frc.robot.subsystems.Intake.Range;
import frc.robotnew.subsystems.Arm.ArmToFloor;
import frc.robotnew.subsystems.Intake.EjectDirection;

import java.lang.annotation.Target;
import java.nio.file.Path;
import java.nio.file.Paths;

public class RobotContainer {

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(Constants.MaxSpeed);

    /**
	 * <strong>Drivetrain Subsystem</strong>
	 */
    public final SwerveDriveTrain drivetrain = TunerConstants.createDrivetrain();
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

    public RobotContainer() {
        SmartDashboard.putString("Auto Step", "None");

        /**
         * Brings the arm from its' current position to a target position.
         * 
         * @param armTarget The target position for the arm base.
         * @param wristTarget The target position for the wrist.
         * 
         * @param endState The state to set the subsystem to when the command ends.
         */
        class ArmToPose extends Command {

            // Target positions
            Angle wristTarget;
            Angle armTarget;

            public ArmToPose(Angle armTarget, Angle wristTarget) {
                super();
                this.armTarget = armTarget;
                this.wristTarget = wristTarget;

                addRequirements(arm);
            }

            /**
             * Called when the command is scheduled.
             */
            @Override
            public void initialize() {
                arm.targetState = Arm.TargetState.AUTO;
                
                arm.setArmMotorPosition(armTarget.magnitude());
                arm.setWristMotorPosition(wristTarget.magnitude());
            }

            /**
             * Called during execution of the command.
             */
            @Override
            public void execute() {
                arm.targetState = Arm.TargetState.AUTO;
            }

            /**
             * Returns true when the command should end.
             */
            @Override
            public boolean isFinished() {
                return (
                    arm.isArmAtPosition(armTarget, Rotations.of(0.03)) &&
                    arm.isWristAtPosition(wristTarget, Rotations.of(0.03))
                );
            }

            /**
             * Called when the command ends.
             */
            @Override
            public void end(boolean interrupted) {
                arm.targetState = Arm.TargetState.AUTO;
            }
        }

        /**
         * Moves the arm to the intake position.
         */
        class ArmToIntake extends ArmToPose {
            public ArmToIntake() {
                super(Constants.ArmPositions.FLOOR, Constants.WristPositions.INTAKE);
            }

            @Override
            public void end(boolean interrupted) {
                super.end(interrupted);
                arm.pauseArmMotor();
            }
        }

        /**
         * Moves the arm to the intake position.
         */
        class ArmExitStart extends ArmToPose {
            public ArmExitStart() {
                super(Constants.ArmPositions.EXIT_STARTING, Constants.WristPositions.STOW);
            }

            //! Josiah's suggested changes:
            // @Override
            // public boolean isFinished() {
            //     return (
            //         arm.getBaseMotor() > Constants.ArmPositions.EXIT_STARTING + Rotations.of(0.03)
            //     );
            // }
        }

        /**
         * Moves the arm to the intake position.
         */
        class ArmToL3 extends ArmToPose {
            public ArmToL3() {
                super(Constants.ArmPositions.LVL3, Constants.WristPositions.LVL3);
            }
        }

        /**
         * Moves the arm to the intake position.
         */
        class ArmToL2 extends ArmToPose {
            public ArmToL2() {
                super(Constants.ArmPositions.LVL2, Constants.WristPositions.LVL2);
            }
        }

        class ScoreL3 extends Command {
            @Override
            public void initialize() {
                arm.targetState = Arm.TargetState.AUTO;
                SmartDashboard.putString("Auto Step", "GoToL3");
                superstructure.targetState = Superstructure.TargetState.GO_TO_LVL3;
            }

            @Override
            public void execute() {
                arm.targetState = Arm.TargetState.AUTO;
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
        //! You have two registered commands about moving the arm to L3, one here and one farther down.
        // This one is the one that calls `ScoreL3()` and is the one that gets called in the pathplanner auto before getting Algae
        // Consider renaming this one and moving it down with the others.
        NamedCommands.registerCommand("GoToL3", new ScoreL3());
        
        class EjectCoral extends Command {

            private double speed;

            public EjectCoral(EjectDirection direction) {
                switch (direction) {
                    case FORWARD:
                        speed = -Constants.IntakeSpeeds.MAX;
                        break;
                    case REVERSE:
                        speed = Constants.IntakeSpeeds.MAX;
                        break;
                    default:
                        speed = -Constants.IntakeSpeeds.MAX;
                        break;
                }

                addRequirements(intake);
            }

            /**
             * Run intake motors in reverse to eject coral.
             */
            @Override
            public void execute() {
                intake.targetState = Intake.TargetState.AUTO;
                intake.setLeftIntake(speed);
                intake.setRightIntake(speed);
                intake.setRollers(speed);
            }

            /**
             * Turn off intake and set final state.
             */
            @Override
            public void end(boolean interrupted) {
                intake.targetState = Intake.TargetState.DEFAULT;
                intake.setLeftIntake(0);
                intake.setRightIntake(0);
                intake.setRollers(0);
            }

            /**
             * Finish when the stop range and the front range are no longer triggered.
             */
            @Override
            public boolean isFinished() {
                return !(intake.getRange(Range.STOP) && intake.getRange(Range.FRONT));
            }
        }

        // BTW (i think) this is equivalent to:
        // new RunCommand(() -> {
        //     intake.targetState = Intake.TargetState.AUTO;
        //     intake.setLeftIntake(1);
        //     intake.setRightIntake(1);
        //     intake.setRollers(1);
        // }).until((intake.getRange(Range.STOP) || intake.getRange(Range.FRONT)));
        // or
        // private Command removeAlgae() {
        //     return this.run(() -> {
        //         intake.targetState = Intake.TargetState.AUTO;
        //         intake.setLeftIntake(1);
        //         intake.setRightIntake(1);
        //         intake.setRollers(1);
        //     }).until((intake.getRange(Range.STOP) || intake.getRange(Range.FRONT)));
        // }
        class RemoveAlgae extends Command {
            /**
             * Run intake motors in reverse to eject coral.
             */
            @Override
            public void execute() {
                intake.targetState = Intake.TargetState.AUTO;
                intake.setLeftIntake(1);
                intake.setRightIntake(1);
                intake.setRollers(1);
            }

            @Override
            public boolean isFinished() {
                return (intake.getRange(Range.STOP) || intake.getRange(Range.FRONT));
            }
        }

        // This also should be equivalent to:
        // new InstantCommand(() -> {
        //     intake.setLeftIntake(0);
        //     intake.setRightIntake(0);
        //     intake.setRollers(0);
        // })
        // or
        // private Command cutIntake() {
        //    return this.runOnce(() -> {
        //     intake.setLeftIntake(0);
        //     intake.setRightIntake(0);
        //     intake.setRollers(0);
        // });
        //}
        class CutIntake extends Command {
            @Override
            public void initialize() {
                intake.setLeftIntake(0);
                intake.setRightIntake(0);
                intake.setRollers(0);
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        }

        NamedCommands.registerCommand("EjectCoral", new InstantCommand(() -> {
            SmartDashboard.putString("Auto Step", "EjectCoral");
            superstructure.targetState = Superstructure.TargetState.EJECT;
        }));
        NamedCommands.registerCommand("EjectCoralReverse", new EjectCoral(EjectDirection.REVERSE));
        NamedCommands.registerCommand("EjectCoralForward", new EjectCoral(EjectDirection.FORWARD));
        NamedCommands.registerCommand("RemoveAlgae", new RemoveAlgae());
        NamedCommands.registerCommand("ArmToL3", new ArmToL3());
        //! For the Algae removal, consider:
        // NamedCommands.registerCommand("RemoveHighAlgae", new ArmToL3.andThen(new RemoveAlgae));
        NamedCommands.registerCommand("ArmToL2", new ArmToL2());
        NamedCommands.registerCommand("ArmToIntake", new ArmToIntake());
        NamedCommands.registerCommand("CutIntake", new CutIntake());

        NamedCommands.registerCommand("ExitStart", new ArmExitStart().andThen(new ArmToIntake()));
        NamedCommands.registerCommand("ExitStartToL2", new ArmExitStart().andThen(new ArmToL2()));

        NamedCommands.registerCommand("ToDefault", new InstantCommand(() -> {
            SmartDashboard.putString("Auto Step", "ToDefault");
            superstructure.targetState = Superstructure.TargetState.DEFAULT;
        }));
        NamedCommands.registerCommand("ToAlgaeL2", new InstantCommand(() -> {
            SmartDashboard.putString("Auto Step", "ToAlgaeL2");
            superstructure.targetState = Superstructure.TargetState.ALGAE_L2_AUTO;
        }));
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Selected Auto", autoChooser);
        // autoChooser.setDefaultOption("Diagonal GGAuto");
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

        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> brake));
        
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
        RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        Constants.driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        Constants.driver.b().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-Constants.driver.getLeftY(), -Constants.driver.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y. Note that each routine should be run exactly once in a single log.
        Constants.driver.back().and(Constants.driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        Constants.driver.back().and(Constants.driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        Constants.driver.start().and(Constants.driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        Constants.driver.start().and(Constants.driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on down POV press.
        Constants.driver.povDown().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // return Commands.print("No autonomous command configured");
        // return new PathPlannerAuto("New Auto");
        return autoChooser.getSelected();
    }
}