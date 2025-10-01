// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Fahrenheit;

import java.util.ArrayList;
import java.util.Dictionary;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.List;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.Tracks;
import frc.robot.LimelightHelpers.PoseEstimate;
import edu.wpi.first.units.BaseUnits.*;
import frc.robot.subsystems.*;

// https://prod.liveshare.vsengsaas.visualstudio.com/join?A272BB5848034C8CE6F916D34B585613FF9F

public class Robot extends TimedRobot {

    private Command m_autonomousCommand;
    public boolean utilizeLimelight = true;

    Field2d llestamation = new Field2d();
    Field2d robotPose = new Field2d();
    Field2d nearestPoseField = new Field2d();

    double slider_value = 0;
    

    private final RobotContainer m_robotContainer;

    public Robot() {
        m_robotContainer = new RobotContainer();
        SmartDashboard.putData("Limelight Pose", llestamation);
        SmartDashboard.putData("Robot Pose", robotPose);
        SmartDashboard.putData("Navigate Target Pose", nearestPoseField);

        // ! Memory Error; implement when ur system doesn't suck.
        // Logger.recordMetadata("Hotwire Project", "2026"); // Set a metadata value

        // if (isReal()) {
        // Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
        // Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        // } else {
        // setUseTiming(false); // Run as fast as possible
        // String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from
        // AdvantageScope (or prompt the user)
        // Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        // Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath,
        // "_sim"))); // Save outputs to a new log
        // }

        // Logger.start(); // Start logging! No more data receivers, replay sources, or
        // metadata values may be added.
    }

    // Non-functional; Implement later
    Orchestra m_orchestra = new Orchestra(
        Filesystem.getDeployDirectory() + "/orchestra/mac.chrp"
    );

    @Override
    public void robotInit() {
      PathfindingCommand.warmupCommand().schedule();

      // var status = m_orchestra.loadMusic("track.chrp");

      // m_orchestra.addInstrument(m_robotContainer.arm.baseMotor());
      // m_orchestra.addInstrument(m_robotContainer.arm.wristMotor());
      // m_orchestra.addInstrument(m_robotContainer.drivetrain.getModule(0).getDriveMotor());
      // m_orchestra.addInstrument(m_robotContainer.drivetrain.getModule(1).getDriveMotor());
      // m_orchestra.addInstrument(m_robotContainer.drivetrain.getModule(2).getDriveMotor());
      // m_orchestra.addInstrument(m_robotContainer.drivetrain.getModule(3).getDriveMotor());
      // m_orchestra.play(); // TODO Make orchestra function.
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        Angle value = m_robotContainer.arm.arm_encoder.getPosition().getValue();
        SmartDashboard.putString("Arm Position", value.toString());

        // Temperature temp = m_robotContainer.arm.getBaseMotor().getDeviceTemp().getValue();
        // SmartDashboard.putNumber("Arm base Motor Temperature", (temp.in(Fahrenheit)));

        Intake.TargetState target = m_robotContainer.intake.targetState;
        Intake.SystemState current = m_robotContainer.intake.currentState;

        SmartDashboard.putString("Intake TargetState", target.toString());
        SmartDashboard.putString("Intake SystemState", current.toString());

        SmartDashboard.putString("Arm TargetState", (m_robotContainer.arm.targetState).toString());
        SmartDashboard.putString("Arm SystemState", (m_robotContainer.arm.currentState).toString());

        SmartDashboard.putString("Drivetrain TargetState", (m_robotContainer.drivetrain.targetState).toString());
        SmartDashboard.putString("Drivetrain SystemState", (m_robotContainer.drivetrain.currentState).toString());

        SmartDashboard.putString("Superstructure Current State", m_robotContainer.superstructure.systemState.toString());
        SmartDashboard.putString("Superstructure Target State", m_robotContainer.superstructure.targetState.toString());
        
        for (Intake.Range range : Intake.Range.values()) {
          Boolean measurement = m_robotContainer.intake.getMeasurement(range);
          SmartDashboard.putBoolean(range.toString() + " CANrange", measurement);
        }

        SmartDashboard.putBoolean("Is Route Complete", m_robotContainer.superstructure.routeComplete);

        robotPose.setRobotPose(m_robotContainer.drivetrain.getState().Pose);
        
        if (utilizeLimelight) {
            List<PoseEstimate> measurements = new ArrayList<PoseEstimate>();

            SwerveDriveState driveState;
            double headingDeg;
            double omegaRPS;
            PoseEstimate limelightMeasurement;


            /*
            //  * `limelight-one` is back.
             * `limelight-two` is front.
             */
            String[] limelightNames = {"limelight-two", "limelight-one"};

            for (String limelight : limelightNames) {

              driveState = m_robotContainer.drivetrain.getState();
              headingDeg = driveState.Pose.getRotation().getDegrees();
              omegaRPS = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
              LimelightHelpers.setPipelineIndex(limelight, 0);
              LimelightHelpers.SetRobotOrientation(limelight, headingDeg, 0, 0, 0, 0, 0);
              limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);

              if ((limelightMeasurement != null) && (limelightMeasurement.tagCount > 0) && (Math.abs(omegaRPS) < 2)) {
                  measurements.add(limelightMeasurement);
                  break;
              }
            }
            if (measurements.size() > 0) {
              int bestTagCount = 0;  
              PoseEstimate bestMeasurement = measurements.get(0);
              for (PoseEstimate measurement : measurements) {
                if (measurement.tagCount > bestTagCount) {
                  bestTagCount = measurement.tagCount;
                  bestMeasurement = measurement;
                }
              }
              m_robotContainer.drivetrain.addVisionMeasurement(bestMeasurement.pose,
                bestMeasurement.timestampSeconds);
              llestamation.setRobotPose(bestMeasurement.pose);
            }
            // Pose2d poseSum = new Pose2d();
            // for (PoseEstimate measurement : measurements) {
            //   poseSum = poseSum.plus(new Transform2d(measurement.pose.getTranslation(), new Rotation2d()));
            // }
            // poseSum = poseSum.div(measurements.size());

            // llestamation.setRobotPose(poseSum);

      }

      Translation2d end = new Translation2d(0, 0);

      // SmartDashboard.putNumber("Distance to Score",
      //     end.getDistance(m_robotContainer.drivetrain.getState().Pose.getTranslation()));

      // nearestPoseField.setRobotPose(Constants.nearestTagPose(m_robotContainer.drivetrain.getState().Pose).get());
      if (m_robotContainer.drivetrain.nearestPose != null) {
        nearestPoseField.setRobotPose(m_robotContainer.drivetrain.nearestPose);
      }
    }

    // Command pfc = AutoBuilder.pathfindToPose(new Pose2d(0, 0, new Rotation2d(0,
    // 0)), constraints);

    // public double getClosestTag(int[] IDs) {
    // Translation2d end = new Translation2d(0, 0); // None Pose

    // // red = 6 - 11
    // // blue =

    // for (int ID = 0; ID < IDs.length; ID++) {
    // int id = IDs[ID];

    // Translation2d tag = taglayout.getTagPose(id).orElse(new Pose3d(end.getX(),
    // end.getY(), 0, new Rotation3d(0, 0, 0))).getTranslation().toTranslation2d();
    // }

    // TODO
    // return
    // tag.getDistance(m_robotContainer.drivetrain.getState().Pose.getTranslation());
    // var apriltags = taglayout.getTags();
    // Pose2d[] tag_poses = new Pose2d[apriltags.size()];
    // for (var tag : apriltags) {
    // tag_poses = tag.pose;
    // }
    // return m_robotContainer.drivetrain.getState().Pose.nearest(apriltags);
    // }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
      m_robotContainer.superstructure.targetState = Superstructure.TargetState.AUTONOMOUS;
      m_autonomousCommand = m_robotContainer.getAutonomousCommand();

      if (m_autonomousCommand != null) {
        m_autonomousCommand.schedule();
      }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
      m_robotContainer.superstructure.targetState = Superstructure.TargetState.DEFAULT;
      if (m_autonomousCommand != null) {
        m_autonomousCommand.cancel();
      }
    }

    @Override
    public void teleopPeriodic() {
      SmartDashboard.putString("Drivetrain State", 
        m_robotContainer.drivetrain.currentState.toString()
      );
      SmartDashboard.putString("Superstructure State", 
        m_robotContainer.superstructure.systemState.toString()
      );
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
      CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
