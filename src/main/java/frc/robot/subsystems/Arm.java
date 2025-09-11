package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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
import frc.robot.Constants.WristPositions;
import frc.robot.commands.CommandWrapper;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

public class Arm extends SubsystemBase {
	
	public enum TargetState {
		/**
		 * Robot is offline, move to starting positons.
		 */
		STOP, 
		IDLE,
		PRACTICE,

		INTAKE,
		HOME,
		START,
	}

	public TargetState targetState = TargetState.STOP;

	public enum SystemState {
		/**
		 * Robot is offline, move to starting positons.
		 */
		STOPPED, 
		HOMING_SHOULDER,
		HOMING_WRIST,
		IDLING,
		PRACTICING,

		INTAKING,
		HOMING,
		STARTING,
	}

	public Angle targetArmPosition   = Constants.ArmPositions.START;
	public Angle targetWristPosition = Constants.WristPositions.START;

	public SystemState currentState = SystemState.STOPPED;
	public SystemState getSystemState() {
		return currentState;
	}
	public SystemState previousSuperState;

	private final TalonFX m_arm_base;
	private final TalonFX m_arm_wrist;

	public final CANcoder arm_encoder;

	public Arm() {
		m_arm_base = new TalonFX(Constants.MotorIDs.arm_base_id);
		m_arm_wrist = new TalonFX(Constants.MotorIDs.arm_wrist_id);

		arm_encoder = new CANcoder(Constants.CANcoderIDs.arm_base_encoder_id);
	}
	
	public void periodic() {handleStateTransitions(); applyStates();}

	private void handleStateTransitions() {

		switch (targetState) {
			case STOP:
				currentState = SystemState.STOPPED;
				break;
			case PRACTICE:
				currentState = SystemState.PRACTICING;
				break;
			case INTAKE:
				currentState = SystemState.INTAKING;
				break;
			case HOME:
				currentState = SystemState.HOMING;
				break;
			default:
				currentState = SystemState.STOPPED;
				break;
		}
	}

	private void applyStates() {

		switch (currentState) {
			case STOPPED:
				break;
			case INTAKING:
				pauseArmMotor();
				setWristMotorPosition(Constants.WristPositions.INTAKE.magnitude());
				break;
			case HOMING:
				pauseArmMotor();
				setWristMotorPosition(Constants.WristPositions.STOW.magnitude());
				break;
			case PRACTICING:
				pauseArmMotor();
				setWristMotorPosition(WristPositions.STOW.magnitude());
				break;
			default:
				break;
		}
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
     * Takes a factor from <strong>-1 to 1</strong> and moves 
	 * the base motor at the correct speed.
     *
     * @param speed Factor from -1 to 1
     */
	public void setBaseMotor(double speed) {
		m_arm_base.set(speed);
	}

	/**
     * Takes a factor from <strong>-1 to 1</strong> and moves 
	 * the wrist motor at the correct speed.
     *
     * @param speed Factor from -1 to 1
     */
	public void setWristMotor(double speed) {
		m_arm_wrist.set(speed);
	}

	/**
     * Takes a factor from <strong>-1 to 1</strong> and moves 
	 * the shoulder to the appropriate angle.
     *
     * @param position Factor from -1 to 1
     */
	public void setArmMotorPosition(double position) {
		m_arm_base.setControl(Constants.Requests.MOTIONMAGIC.withPosition(position));
	}

	/**
     * Takes a factor from <strong>-1 to 1</strong> and moves 
	 * the wrist to the appropriate angle.
     *
     * @param position Factor from -1 to 1
     */
	public void setWristMotorPosition(double position) {
		m_arm_wrist.setControl(Constants.Requests.MOTIONMAGIC.withPosition(position));
	}

	public void pauseWristMotor() {
		final VoltageOut request = Constants.Requests.ZERO;
		m_arm_wrist.setControl(request);
	}

	public void pauseArmMotor() {
		final VoltageOut request = Constants.Requests.ZERO;
		m_arm_base.setControl(request);
	}
}
