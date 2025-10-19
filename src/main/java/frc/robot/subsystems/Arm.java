package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
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
import frc.robot.commands.CommandWrapper;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robotnew.Constants;
import frc.robotnew.Constants.WristPositions;

public class Arm extends SubsystemBase {
	
	public enum TargetState {
		/**
		 * Robot is offline, move to starting positons.
		 */
		STOP, 
		IDLE,
		EXIT_STARTING_POSE,
		PRACTICE,

		INTAKE,
		DEFAULT,
		START,

		SCORE_LVL2,
		SCORE_LVL3,

		TAKE_ALGAE_L3,
		REMOVE_ALGAE_L3,
		TAKE_ALGAE_L2,
		RUNTOPOSE
	}

	public TargetState targetState = TargetState.STOP;

	public enum SystemState {
		/**
		 * Robot is offline, move to starting positons.
		 */
		STOPPED, 
		// HOMING_SHOULDER,
		// HOMING_WRIST,
		IDLING,
		EXITING_STARTING_POSE,
		PRACTICING,

		INTAKING,
		HOMING,
		HOME,

		SCORING_LVL2,
		SCORING_LVL3,

		TAKING_ALGAE_L3,
		REMOVING_ALGAE_L3,
		TAKING_ALGAE_L2,
		RUNTOPOSE
	}

	public boolean IS_ARM_AT_EXIT_STARTING_POSITION;

	public SystemState currentState = SystemState.STOPPED;
	public SystemState getSystemState() {
		return currentState;
	}
	public SystemState previousSuperState;

	private final TalonFX m_arm_base;
	private final TalonFX m_arm_follower;
	private final TalonFX m_arm_wrist;

	public final CANcoder arm_encoder;

	public Arm() {
		m_arm_base     = new TalonFX(Constants.MotorIDs.arm_base_id);
		m_arm_follower = new TalonFX(Constants.MotorIDs.arm_follower_id);
		m_arm_follower.setControl(new Follower(Constants.MotorIDs.arm_base_id, true));
		// Master is counter-clockwise and follower is clockwise.
		m_arm_wrist    = new TalonFX(Constants.MotorIDs.arm_wrist_id);

		arm_encoder    = new CANcoder(Constants.CANcoderIDs.arm_base_encoder_id);
	}
	
	public void periodic() {
		IS_ARM_AT_EXIT_STARTING_POSITION = (
			isArmAtPosition(Constants.ArmPositions.EXIT_STARTING, Rotations.of(0.015)) &&
			(getBaseMotor().gt(Rotations.of(0.05))) && (targetState == TargetState.EXIT_STARTING_POSE)
		);

		handleStateTransitions(); applyStates();
		
	}

	public Angle wristTarget = Constants.WristPositions.LVL2;
	public Angle armTarget = Constants.ArmPositions.LVL2;

	private void handleStateTransitions() {

		switch (targetState) {
			case STOP:
				currentState = SystemState.STOPPED;
				break;
			case RUNTOPOSE:
				currentState = SystemState.RUNTOPOSE;
				break;
			case PRACTICE:
				currentState = SystemState.PRACTICING;
				break;
			case INTAKE:
				currentState = SystemState.INTAKING;
				break;
			case DEFAULT:
				currentState = (!isArmAtPosition(Constants.ArmPositions.FLOOR, Rotations.of(0.08))) 
					? SystemState.HOMING : SystemState.HOME;
				break;
			case SCORE_LVL2:
				currentState = SystemState.SCORING_LVL2;
				break;
			case SCORE_LVL3:
				currentState = SystemState.SCORING_LVL3;
				break;
			case EXIT_STARTING_POSE:
				currentState = SystemState.EXITING_STARTING_POSE;
				break;
			case TAKE_ALGAE_L3:
				currentState = SystemState.TAKING_ALGAE_L3;
				break;
			case REMOVE_ALGAE_L3:
				currentState = SystemState.REMOVING_ALGAE_L3;
				break;
			case TAKE_ALGAE_L2:
				currentState = SystemState.TAKING_ALGAE_L2;
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
			case RUNTOPOSE:
				setArmMotorPosition(armTarget.magnitude());
				setWristMotor(wristTarget.magnitude());
				break;
			case HOMING:
				setArmMotorPosition(Constants.ArmPositions.FLOOR.magnitude());
				// setArmMotorToConstantVoltageBackward();
				setWristMotorPosition(Constants.WristPositions.STOW.magnitude());
				break;
			case HOME:
				pauseArmMotor();
				setWristMotorPosition(Constants.WristPositions.STOW.magnitude());
				break;
			case PRACTICING:
				pauseArmMotor();
				setWristMotorPosition(Constants.WristPositions.STOW.magnitude());
				break;
			case SCORING_LVL2:
				setArmMotorPosition(Constants.ArmPositions.LVL2.magnitude());
				setWristMotorPosition(Constants.WristPositions.LVL2.magnitude());
				break;
			case SCORING_LVL3:
				setArmMotorPosition(Constants.ArmPositions.LVL3.magnitude());
				setWristMotorPosition(Constants.WristPositions.LVL3.magnitude());
				break;
			case EXITING_STARTING_POSE:
				setArmMotorPosition(Constants.ArmPositions.EXIT_STARTING.magnitude());
				setWristMotorPosition(Constants.WristPositions.STOW.magnitude());
				break;
			case TAKING_ALGAE_L3:
				setArmMotorPosition(Constants.ArmPositions.TAKE_ALGAE_L3.magnitude());
				setWristMotorPosition(Constants.WristPositions.TAKE_ALGAE_L3.magnitude());
				break;
			case TAKING_ALGAE_L2:
				setArmMotorPosition(Constants.ArmPositions.TAKE_ALGAE_L2.magnitude());
				setWristMotorPosition(Constants.WristPositions.TAKE_L2.magnitude());
				break;
			case REMOVING_ALGAE_L3:
				setArmMotorPosition(Constants.ArmPositions.TAKE_ALGAE_L3.magnitude());
				setWristMotorPosition(Constants.WristPositions.REMOVE_ALGAE_L3.magnitude());
				break;
			default:
				break;
		}
	}

	/**
     * Get the base motor.
     */
	public TalonFX baseMotor() {
		return m_arm_base;
	}

	/**
     * Get the follower motor.
     */
	public TalonFX followerMotor() {
		return m_arm_follower;
	}

	/**
     * Get the wrist motor.
     */
	public TalonFX wristMotor() {
		return m_arm_wrist;
	}

	/**
     * Is the arm at an angle and within a given threshold.
     */
	public boolean isArmAtPosition(Angle angle, Angle threshold) {
		// System.out.println("Arm Difference: " + m_arm_base.getPosition().getValue().minus(angle).magnitude());
		return Math.abs(m_arm_base.getPosition().getValue().minus(angle).magnitude()) < threshold.magnitude();
	}

	/**
     * Is the wrist at an angle and within a given threshold.
     */
	public boolean isWristAtPosition(Angle angle, Angle threshold) {
		// System.out.println("Wrist Difference: " + m_arm_wrist.getPosition().getValue().minus(angle).magnitude());
		return Math.abs(m_arm_wrist.getPosition().getValue().minus(angle).magnitude()) < threshold.magnitude();
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
	 * the shoulder to the appropriate angle.
     *
     * @param position Factor from -1 to 1
     */
	public void setArmMotorToConstantVoltageBackward() {
		m_arm_base.setControl(Constants.Requests.VOLTAGE_BACKWARD);
	}

	/**
     * Takes a factor from <strong>-1 to 1</strong> and moves 
	 * the wrist to the appropriate angle.
     *
     * @param position Factor from -1 to 1
     */
	public void setWristMotorPosition(double position) {
		// double feedforward = 0.3349609375 * // Gravity Constant
		// Math.cos(
		// 	m_arm_wrist.getPosition().getValueAsDouble() + 
		// 	m_arm_base.getPosition().getValueAsDouble());
		// SmartDashboard.putNumber("feedforward", feedforward);
		
		// m_arm_wrist.setControl(Constants.Requests.MOTIONMAGIC.withPosition(position)
		// 	.withFeedForward(feedforward));
		SmartDashboard.putNumber("Arm Target", position);
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

	/**
     * Get the base motor.
     */
	public Angle getBaseMotor() {
		return m_arm_base.getPosition().getValue();
	}

	/**
     * Get the follower motor.
     */
	public Angle getFollowerMotor() {
		return m_arm_follower.getPosition().getValue();
	}

	/**
     * Get the wrist motor.
     */
	public Angle getWristMotor() {
		return m_arm_wrist.getPosition().getValue();
	}
}
