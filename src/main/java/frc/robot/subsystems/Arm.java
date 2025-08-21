package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
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
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.commands.CommandWrapper;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

public class Arm extends SubsystemBase {
	
	public enum TargetState {
		STOPPED, 
		DEFAULT, 
		SCORE_LEFT, 
		SCORE_RIGHT, 
		GET_CORAL
	}

	public TargetState targetState = TargetState.STOPPED;

	public enum SystemState {
		STOPPED,
		NO_CORAL, 
		SCORING_CORAL_LEFT,
		SCORING_CORAL_RIGHT, 
		INTAKING_CORAL, 
		HOLDING_CORAL
	}

	public SystemState currentState = SystemState.STOPPED;
	public SystemState previousSuperState;

	private final TalonFX m_arm_base;
	private final TalonFX m_arm_wrist;

	public Arm() {
		m_arm_base = new TalonFX(Constants.MotorIDs.arm_base_id);
		m_arm_wrist = new TalonFX(Constants.MotorIDs.arm_wrist_id);
	}

	/**
     * Get the base motor.
     */
	public TalonFX getBaseMotor() {
		return m_arm_base;
	}

	/**
     * Get the wrist motor.
     */
	public TalonFX getWristMotor() {
		return m_arm_wrist;
	}

    /**
     * Takes a factor from <strong>-1 to 1</strong> and runs 
	 * the base motor at the appropriate speed.
     *
     * @param speed Factor from -1 to 1
     */
	public void runBaseMotor(double speed) {
		m_arm_base.set(speed);
	}

	/**
     * Takes a factor from <strong>-1 to 1</strong> and runs 
	 * the wrist motor at the appropriate speed.
     *
     * @param speed Factor from -1 to 1
     */
	public void runWristMotor(double speed) {
		m_arm_wrist.set(speed);
	}
}
