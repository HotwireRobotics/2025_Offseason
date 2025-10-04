// package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.*;

// import java.util.function.Supplier;

// import com.ctre.phoenix6.Orchestra;
// import com.ctre.phoenix6.SignalLogger;
// import com.ctre.phoenix6.Utils;
// import com.ctre.phoenix6.controls.ControlRequest;
// import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.controls.MusicTone;
// import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix6.controls.VoltageOut;
// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.hardware.CANrange;
// import com.ctre.phoenix6.hardware.ParentDevice;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.hardware.TalonFXS;
// import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
// import com.ctre.phoenix6.swerve.SwerveModuleConstants;
// import com.ctre.phoenix6.swerve.SwerveRequest;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;

// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.Distance;
// import edu.wpi.first.units.Units.*;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj.Filesystem;
// import edu.wpi.first.wpilibj.Notifier;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Subsystem;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// import frc.robot.Constants;
// import frc.robot.Constants.WristPositions;
// import frc.robot.commands.CommandWrapper;
// import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

// public class Climber extends SubsystemBase {
	
// 	public enum TargetState {
// 		/**
// 		 * Robot is offline, move to starting positons.
// 		 */
// 		STOP,
// 	}

// 	public TargetState targetState = TargetState.STOP;

// 	public enum SystemState {
// 		/**
// 		 * Robot is offline, move to starting positons.
// 		 */
// 		STOPPED,
// 	}

// 	private final TalonFX m_winch;
// 	private final DigitalInput limit_switch;

// 	public Climber() {
// 		m_winch = new TalonFX(Constants.MotorIDs.winch_id);
// 		limit_switch = new DigitalInput(0);
// 	}

// 	public boolean getLimitSwitch() {
// 		return limit_switch.get();
// 	}

// 	public TalonFX winchMotor() {
// 		return m_winch;
// 	}

// 	public Angle getWinchMotor() {
// 		return m_winch.getPosition().getValue();
// 	}
// }