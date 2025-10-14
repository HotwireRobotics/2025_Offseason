// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robotnew;

import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robotnew.LimelightHelpers.PoseEstimate;
import frc.robotnew.subsystems.Superstructure;

public class Robot extends TimedRobot {

    private Command m_autonomousCommand;
    private final RobotContainer m_robotContainer;

    private final Field2d llEstimation = new Field2d();
    private final Field2d robotPose = new Field2d();
    private final Field2d nearestPoseField = new Field2d();

    public boolean utilizeLimelight = true;

    public Robot() {
        m_robotContainer = new RobotContainer();

        SmartDashboard.putData("Limelight Pose", llEstimation);
        SmartDashboard.putData("Robot Pose", robotPose);
        SmartDashboard.putData("Navigate Target Pose", nearestPoseField);

        for (String limelight : Constants.LIMELIGHT_NAMES) {
            SmartDashboard.putBoolean(limelight + " detecting", false);
        }
    }

    @Override
    public void robotInit() {
        PathfindingCommand.warmupCommand().schedule();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        SmartDashboard.putBoolean("Robot Enabled", DriverStation.isEnabled());

        robotPose.setRobotPose(m_robotContainer.drivetrain.getState().Pose);

        if (utilizeLimelight) {
            List<PoseEstimate> measurements = new ArrayList<>();
            boolean detectedFlag = false;

            for (String limelight : Constants.LIMELIGHT_NAMES) {
                var driveState = m_robotContainer.drivetrain.getState();
                double headingDeg = driveState.Pose.getRotation().getDegrees();
                double omegaRPS = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

                /**
                 * Pipeline 0 is for the red side,
                 * Pipeline 1 is for the blue side.
                 */
                LimelightHelpers.setPipelineIndex(limelight, (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? 0 : 1);
                // LimelightHelpers.setPipelineIndex(limelight, 0);
                LimelightHelpers.SetRobotOrientation(limelight, headingDeg, 0, 0, 0, 0, 0);

                PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);

                if (limelightMeasurement != null &&
                    limelightMeasurement.tagCount > 0 &&
                    Math.abs(omegaRPS) < 2 &&
                    limelightMeasurement.avgTagDist < Constants.MaxDetectionRadius.in(Meters)) {

                    measurements.add(limelightMeasurement);
                    SmartDashboard.putBoolean(limelight + " detecting", true);
                    detectedFlag = true;

                    m_robotContainer.drivetrain.addVisionMeasurement(
                        limelightMeasurement.pose,
                        limelightMeasurement.timestampSeconds
                    );
                    llEstimation.setRobotPose(limelightMeasurement.pose);
                } else {
                    SmartDashboard.putBoolean(limelight + " detecting", false);
                }
            }

            if (detectedFlag) {
                for (String limelightName : Constants.LIMELIGHT_NAMES) {
                    LimelightHelpers.setLEDMode_ForceOn(limelightName);
                }
            } else {
                for (String limelightName : Constants.LIMELIGHT_NAMES) {
                    LimelightHelpers.setLEDMode_ForceOff(limelightName);
                }
            }
        }

        if (m_robotContainer.drivetrain.nearestPose != null) {
            nearestPoseField.setRobotPose(m_robotContainer.drivetrain.nearestPose);
        }
    }

    @Override
    public void autonomousInit() {
        m_robotContainer.superstructure.setSysState(Superstructure.State.AUTO);
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void teleopInit() {
        m_robotContainer.superstructure.setSysState(Superstructure.State.TELEOP);
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }
}
