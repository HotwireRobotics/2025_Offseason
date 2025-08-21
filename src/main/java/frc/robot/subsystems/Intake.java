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

public class Intake extends SubsystemBase {
	
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

	private final CANrange r_right;
	private final CANrange r_left;
	private final CANrange r_front;
	private final CANrange r_stop;

	public enum Range {
		RIGHT, LEFT, FRONT, STOP
	}

	/**
	 * <strong>Righthand wheel intake relative to robot.</strong>
	 */
	private final TalonFXS d_right_intake;
	/**
	 * <strong>Lefthand wheel intake relative to robot.</strong>
	 */
	private final TalonFXS d_left_intake;

	/**
	 * <strong>Horizontal Rollers</strong>
	 */
	private final TalonFXS d_rollers;

	public Intake() {
		r_right = new CANrange(Constants.CANrangeIDs.right);
		r_left = new CANrange(Constants.CANrangeIDs.left);
		r_front = new CANrange(Constants.CANrangeIDs.front);
		r_stop = new CANrange(Constants.CANrangeIDs.stop);

		d_right_intake = new TalonFXS(Constants.MotorIDs.right_intake_id);
		d_left_intake = new TalonFXS(Constants.MotorIDs.left_intake_id);

		d_rollers = new TalonFXS(Constants.MotorIDs.rollers_id);
	}

	/**
     * Get the meaurement from one of the CANranges.
     *
     * @param range Get from the <code>Range</code> enum.
     */
	public Distance getMeasurement(Range range) {
		CANrange range_obj;
		switch (range) {
			case RIGHT: range_obj = r_right; break;
			case LEFT: range_obj = r_left; break;
			case FRONT: range_obj = r_front; break;
			case STOP: range_obj = r_stop; break;
			default: return Meters.of(-1); //! Error
		}
		return range_obj.getDistance().getValue();
	}
}
