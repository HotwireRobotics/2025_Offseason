// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Field;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public boolean utilizeLimelight = true;

  Field2d llestamation = new Field2d();

  AprilTagFieldLayout taglayout = AprilTagFields.k2025ReefscapeWelded.loadAprilTagLayoutField();

  

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
    SmartDashboard.putData("Limelight Pose", llestamation);
  }

  @Override
  public void robotInit() {
    PathfindingCommand.warmupCommand().schedule();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 

    if (utilizeLimelight) {
      var driveState = m_robotContainer.drivetrain.getState();
      var headingDeg = driveState.Pose.getRotation().getDegrees();
      var omegaRPS = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
      LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 
                                            0, 0, 
                                            0, 0, 
                                            0);
      var limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("");

      // m_robotContainer.drivetrain.addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestampSeconds);
      if ((limelightMeasurement != null) && (limelightMeasurement.tagCount > 0) && (Math.abs(omegaRPS) < 2))
      {
        m_robotContainer.drivetrain.addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestampSeconds);
        llestamation.setRobotPose(limelightMeasurement.pose);
      }
    }

    Translation2d end = new Translation2d(0, 0);

    SmartDashboard.putNumber("Distance to Score", end.getDistance(m_robotContainer.drivetrain.getState().Pose.getTranslation()));

    
  }

  // Command pfc = AutoBuilder.pathfindToPose(new Pose2d(0, 0, new Rotation2d(0, 0)), constraints);

  // public double getClosestTag(int[] IDs) {
    // Translation2d end = new Translation2d(0, 0); // None Pose

    // // red = 6 - 11
    // // blue = 

    // for (int ID = 0; ID < IDs.length; ID++) {
    //   int id = IDs[ID];

    //   Translation2d tag = taglayout.getTagPose(id).orElse(new Pose3d(end.getX(), end.getY(), 0, new Rotation3d(0, 0, 0))).getTranslation().toTranslation2d();
    // }

    
    // TODO
    // return tag.getDistance(m_robotContainer.drivetrain.getState().Pose.getTranslation());
    // var apriltags = taglayout.getTags();
    // Pose2d[] tag_poses = new Pose2d[apriltags.size()];
    // for (var tag : apriltags) {
    //   tag_poses = tag.pose;
    // }
    // return m_robotContainer.drivetrain.getState().Pose.nearest(apriltags);
  // }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
