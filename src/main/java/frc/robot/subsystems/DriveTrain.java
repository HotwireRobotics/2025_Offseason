package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Tracks;
import frc.robot.commands.CommandGenerator;
import frc.robot.commands.CommandWrapper;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class DriveTrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    private Field2d m_field = new Field2d(); // This is a field

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    public enum TargetState {
		TELEOP_DRIVE,
        AUTONOMOUS,

		NAVIGATE_DOWN_LEFT,
		NAVIGATE_UP_LEFT,
		NAVIGATE_DOWN_RIGHT,
		NAVIGATE_UP_RIGHT,
        NAVIGATE_EXIT_LVL2,

        STOP, 
        IDLE
	}

    public enum SystemState {
		TELEOP_DRIVE,
        AUTONOMOUS,

		NAVIGATING_DOWN_LEFT,
		NAVIGATING_UP_LEFT,
		NAVIGATING_DOWN_RIGHT,
		NAVIGATING_UP_RIGHT,
        NAVIGATING_EXIT_LVL2,

        STOPPED,
        IDLE
	}

    public boolean GO_HOME = false;
    public boolean getDoGoHome() {
        return GO_HOME;
    }

    public Pose2d wantedPose;

    public TargetState targetState = TargetState.TELEOP_DRIVE;

    public SystemState currentState = SystemState.TELEOP_DRIVE;
	public SystemState getSystemState() {
		return currentState;
	}

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
        
        
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public DriveTrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        buildAuto();
        SmartDashboard.putData("field", m_field);
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public DriveTrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        buildAuto();
        SmartDashboard.putData("field", m_field);
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public DriveTrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        buildAuto();
        SmartDashboard.putData("field", m_field);
    }

    public void buildAuto() {
        try {

            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                () -> getState().Pose, 
                this::resetPose, 
                () -> getState().Speeds, 
                (speeds, feedforwards) -> setControl(m_pathApplyRobotSpeeds.withSpeeds(speeds).withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons()).withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                new PPHolonomicDriveController(
                    new PIDConstants(10.0, 0.0, 0.0), // Translation
                    new PIDConstants(7.0, 0.0, 0.0) // Rotation
                ), 
                config, 
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this
            );

        } catch (Exception e) {
            DriverStation.reportError("Autobuilder Failed!", e.getStackTrace());
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        m_field.setRobotPose(getState().Pose);
        
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        handleStateTransitions(); 
        applyStates();
    }

    Command drive_command = applyRequest(() -> Constants.drive.withVelocityX(-(Constants.driver.getLeftY()) * (Constants.driver.x().getAsBoolean() ? Constants.MaxSpeed / 2 : Constants.MaxSpeed)) 
                // Drive forward with
                // negative Y
                // (forward)
        .withVelocityY(-(Constants.driver.getLeftX()) * (Constants.driver.x().getAsBoolean() ? Constants.MaxSpeed / 2 : Constants.MaxSpeed)) // Drive left with negative X (left)
        .withRotationalRate(-Constants.driver.getRightX() * Constants.MaxAngularRate) // Drive counterclockwise with negative X (left)
    );

    private double lerp(double val, double k /* 2 factor */, double m /* 0.1 starting */) {
        double sign = Math.signum(val);
        val = Math.abs(val);
        double i = Math.min(1, Math.max(0, Math.pow(val, k) + m));
        return i * sign;
    }

    public Command navigateCommand;
    private void handleStateTransitions() {
        switch (targetState) {
            case TELEOP_DRIVE:
                currentState = SystemState.TELEOP_DRIVE;
                navigateCommand = null;
                break;
            case STOP:
                currentState = SystemState.STOPPED;
                break;
            case AUTONOMOUS:
                currentState = SystemState.AUTONOMOUS;
                break;
            case NAVIGATE_DOWN_RIGHT:
                if (currentState != SystemState.NAVIGATING_DOWN_RIGHT) {
                    navigateCommand = navigate();
                    navigateCommand.schedule();
                }
				currentState = SystemState.NAVIGATING_DOWN_RIGHT;
				break;
			case NAVIGATE_DOWN_LEFT:
                // TODO add movement to lvl2 scoring position for arm and intake.
                if (currentState != SystemState.NAVIGATING_DOWN_LEFT) {
                    navigateCommand = navigate();
                    navigateCommand.schedule();
                }
				currentState = SystemState.NAVIGATING_DOWN_LEFT;
				break;
			case NAVIGATE_UP_RIGHT:
                if (currentState != SystemState.NAVIGATING_UP_RIGHT) {
                    navigateCommand = navigate();
                    navigateCommand.schedule();
                }
				currentState = SystemState.NAVIGATING_UP_RIGHT;
				break;
			case NAVIGATE_UP_LEFT:
                if (currentState != SystemState.NAVIGATING_UP_LEFT) {
                    navigateCommand = navigate();
                    navigateCommand.schedule();
                }
				currentState = SystemState.NAVIGATING_UP_LEFT;
				break;
            case NAVIGATE_EXIT_LVL2:
                if (currentState != SystemState.NAVIGATING_EXIT_LVL2) {
                    navigateCommand = navigate();
                    navigateCommand.schedule();
                }
                currentState = SystemState.NAVIGATING_EXIT_LVL2;
                break;
            default:
                currentState = SystemState.IDLE;
                break;
        }
    }

    private void applyStates() {
        switch (currentState) {
            case TELEOP_DRIVE:
                drive_command.schedule();
                break;
            case STOPPED:
                idle().schedule();
                break;
            case AUTONOMOUS:
                // Allow Pathplanner control.
                break;
            case NAVIGATING_UP_LEFT:
			case NAVIGATING_UP_RIGHT:
			case NAVIGATING_DOWN_LEFT:
			case NAVIGATING_DOWN_RIGHT:
            case NAVIGATING_EXIT_LVL2:
            case IDLE:
                break;
        }
    }

    public Pose2d nearestPose;
    public Command navigate() {
        Command command;
        switch (targetState) {
            case NAVIGATE_UP_LEFT: 
                nearestPose = Constants.nearestBranchPose(getState().Pose, Tracks.left).get();
                command = AutoBuilder.pathfindToPose(nearestPose, Constants.constraints);
                break;
            case NAVIGATE_DOWN_LEFT:
                nearestPose = Constants.nearestBranchPose(getState().Pose, Tracks.left).get();
                nearestPose = nearestPose.rotateAround(nearestPose.getTranslation(), new Rotation2d(Radians.of(Math.PI)));
                command = AutoBuilder.pathfindToPose(nearestPose, Constants.constraints);
                break;
            case NAVIGATE_UP_RIGHT: 
                nearestPose = Constants.nearestBranchPose(getState().Pose, Tracks.right).get();
                command = AutoBuilder.pathfindToPose(nearestPose, Constants.constraints);
                break;
            case NAVIGATE_DOWN_RIGHT:
                nearestPose = Constants.nearestBranchPose(getState().Pose, Tracks.right).get();
                nearestPose = nearestPose.rotateAround(nearestPose.getTranslation(), new Rotation2d(Radians.of(Math.PI)));
                command = AutoBuilder.pathfindToPose(nearestPose, Constants.constraints);
                break;
            case NAVIGATE_EXIT_LVL2:
                nearestPose = getState().Pose;
                Transform2d backwards = new Transform2d(new Translation2d(Constants.EXIT_DISTANCE.magnitude(), new Rotation2d()), new Rotation2d());
                nearestPose = nearestPose.plus(backwards);
                command = AutoBuilder.pathfindToPose(nearestPose, Constants.constraints).andThen(new InstantCommand(() -> {
                    GO_HOME = true;
                }));
                break;
            default:
                return drive_command;
        }
        return command;
    }

    public Command idle() {
        return applyRequest(() -> new SwerveRequest.Idle());
    }


    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }
}
