package frc.robot.subsystems.drivetrain;

// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.



import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.util.LocalADStarAK;
import frc.robot.Constants.Dimensions;
import frc.robot.Constants.Tracks;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY =
      new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = 54.431;
  private static final double ROBOT_MOI = 6.883;
  private static final double WHEEL_COF = 1.2;
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              TunerConstants.FrontLeft.WheelRadius,
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              TunerConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

      public enum TargetState {
        TELEOP_DRIVE,
            AUTONOMOUS,
    
        NAVIGATE_DOWN_LEFT,
        NAVIGATE_UP_LEFT,
        NAVIGATE_DOWN_RIGHT,
        NAVIGATE_UP_RIGHT,
            NAVIGATE_EXIT_LVL2,
            NAVIGATE_ALGAE,
    
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
            NAVIGATING_ALGAE,
    
            STOPPED,
            IDLE
      }
    
        public boolean GO_HOME = false;
        public boolean getDoGoHome() {
            return GO_HOME;
        }
    
        public boolean ALGAE_POSE = false;
        public boolean getAlgaePose() {
            return ALGAE_POSE;
        }
    
        public Pose2d wantedPose;
    
        public TargetState targetState = TargetState.TELEOP_DRIVE;
    
        public SystemState currentState = SystemState.TELEOP_DRIVE;
      public SystemState getSystemState() {
        return currentState;
      }

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
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
            case NAVIGATE_ALGAE:
                if (currentState != SystemState.NAVIGATING_ALGAE) {
                    navigateCommand = navigate();
                    // if (ALGAE_POSE) {
                    //     navigateCommand.schedule();
                    // } else {
                    //     GO_HOME = true;
                    // }
                    navigateCommand.schedule();
                }
                currentState = SystemState.NAVIGATING_ALGAE;
                break;
            default:
                currentState = SystemState.IDLE;
                break;
        }
    }

    private void applyStates() {
        switch (currentState) {
            case TELEOP_DRIVE:
                driveCommand.schedule();
                break;
            case STOPPED:
                Commands.runOnce(this::stopWithX, this);
                break;
            case AUTONOMOUS:
                // Allow Pathplanner control.
                break;
            case NAVIGATING_UP_LEFT:
			case NAVIGATING_UP_RIGHT:
			case NAVIGATING_DOWN_LEFT:
			case NAVIGATING_DOWN_RIGHT:
            case NAVIGATING_EXIT_LVL2:
            case NAVIGATING_ALGAE:
            case IDLE:
                break;
        }
    }

    public Integer nearestId = -1;
    public Pose2d nearestPose;
    public Command navigate() {
        Command command;
        switch (targetState) {
            case NAVIGATE_UP_LEFT: 
                nearestPose = Constants.nearestBranchPose(getPose(), Tracks.left).get();
                command = AutoBuilder.pathfindToPose(nearestPose, Constants.constraints);
                break;
            case NAVIGATE_DOWN_LEFT:
                nearestPose = Constants.nearestBranchPose(getPose(), Tracks.left).get();
                nearestPose = nearestPose.rotateAround(nearestPose.getTranslation(), new Rotation2d(Radians.of(Math.PI)));
                command = AutoBuilder.pathfindToPose(nearestPose, Constants.constraints);
                break;
            case NAVIGATE_UP_RIGHT: 
                nearestPose = Constants.nearestBranchPose(getPose(), Tracks.right).get();
                command = AutoBuilder.pathfindToPose(nearestPose, Constants.constraints);
                break;
            case NAVIGATE_ALGAE:
                nearestId = Constants.nearestAlgaeId(getPose());
                nearestPose = Constants.taglayout.getTags().get(nearestId - 1).pose.toPose2d();
                Rotation2d rot = Rotation2d.k180deg;
                if (Constants.RED_TAG_IDS.contains(nearestId)) {
                    if ((nearestId % 2) == 0) {
                        rot = Rotation2d.kZero;
                        ALGAE_POSE = false;
                    } else {
                        rot = Rotation2d.k180deg;
                        ALGAE_POSE = true;
                    }
                }
                if (Constants.BLUE_TAG_IDS.contains(nearestId)) {
                    if ((nearestId % 2) == 1) {
                        rot = Rotation2d.kZero;
                        ALGAE_POSE = false;
                    } else {
                        rot = Rotation2d.k180deg;
                        ALGAE_POSE = true;
                    }
                }
                Transform2d offset = new Transform2d(
				    new Translation2d(Dimensions.bumperLength.magnitude() / 2, new Rotation2d()), rot);
		        nearestPose = nearestPose.plus(offset);
                command = AutoBuilder.pathfindToPose(nearestPose, Constants.constraints);
                break;
            case NAVIGATE_DOWN_RIGHT:
                nearestPose = Constants.nearestBranchPose(getPose(), Tracks.right).get();
                nearestPose = nearestPose.rotateAround(nearestPose.getTranslation(), new Rotation2d(Radians.of(Math.PI)));
                command = AutoBuilder.pathfindToPose(nearestPose, Constants.constraints);
                break;
            case NAVIGATE_EXIT_LVL2:
                nearestPose = getPose();
                Transform2d backwards = new Transform2d(new Translation2d(Constants.EXIT_DISTANCE.magnitude(), new Rotation2d()), new Rotation2d());
                nearestPose = nearestPose.plus(backwards);
                command = AutoBuilder.pathfindToPose(nearestPose, Constants.constraints).andThen(new InstantCommand(() -> {
                    GO_HOME = true;
                }));
                break;
            default:
                return driveCommand;
        }
        return command;
    }

    public Command driveCommand = DriveCommands.joystickDrive(
            this,
            () -> -Constants.driver.getLeftY(),
            () -> -Constants.driver.getLeftX(),
            () -> -Constants.driver.getRightX());


  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }
}
