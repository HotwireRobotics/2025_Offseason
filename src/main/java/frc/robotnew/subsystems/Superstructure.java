package frc.robotnew.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robotnew.RobotContainer;
import frc.robotnew.Constants;

public class Superstructure extends SubsystemBase {
	
	//* ----- Initialization -----

	RobotContainer container;

	/**
	 * Constructor for the subsystem.
	 */
	public Superstructure(RobotContainer container) {
		this.container = container;
	}

	//* ----- Periodic -----

	/**
	 * Periodic method for the subsystem.
	 */
	@Override
	public void periodic() {
		
	}

	//* ----- State Management -----
	
	/**
	 * Subsystem state options.
	 */
	public enum State {
		STOPPED,
		TELEOP,
		AUTO,
		PATHFINDING,		
	}

	/** 
	 * Active subsystem state.
	*/
	private State state = State.STOPPED;

	/**
	 * Returns the active subsystem state.	
	 *
	 * @return <code>state</code>
	 */
	public State getSysState() {
		return state;
	}

	/**
	 * Sets the active subsystem state.
	 */
	public void setSysState(State newState) {
		state = newState;
	}

	private Command setSuperState(State newState) {
		return new InstantCommand(() -> setSysState(State.PATHFINDING));
	}

	public Command score(DriveTrain.ScoringPosition pose, Boolean skipNav) {
		Command cmd = setSuperState(State.PATHFINDING)
		.andThen(new ConditionalCommand(
			new ConditionalCommand(
				container.drivetrain.new Navigate(pose)
				, new InstantCommand(), 
			() -> {return !skipNav;})
			.andThen(container.arm.new ArmToL3())
			.andThen(container.intake.new EjectCoral(Intake.EjectDirection.FORWARD)), 
			container.arm.new ArmToL2()
			.andThen(new ConditionalCommand(
				container.drivetrain.new Navigate(pose)
				, new InstantCommand(), 
			() -> {return !skipNav;}))
			.andThen(container.intake.new EjectCoral(Intake.EjectDirection.REVERSE)), 
		() -> {return pose == DriveTrain.ScoringPosition.RIGHT_L3 || pose == DriveTrain.ScoringPosition.LEFT_L3;}))
		.andThen(new WaitCommand(1))
		.andThen(container.arm.new ArmToFloor())
		.andThen(setSuperState(State.TELEOP));
		cmd.addRequirements(container.arm, container.intake, container.drivetrain);
		return cmd;
	}

	public Command exitstart() {
		Command cmd = container.arm.new ArmToL3()
		.andThen(container.arm.new ArmToFloor());
		cmd.addRequirements(container.arm);
		return cmd;
	}

	public Command eject(Intake.EjectDirection direction) {
		Command cmd = container.intake.new EjectCoral(direction);
		cmd.addRequirements(container.intake);
		return cmd;
	}

	public Command stow() {
		Command cmd = container.arm.new ArmToFloor();
		cmd.addRequirements(container.arm);
		return cmd;
	}

	public Command ageL2Arm() {
		Command cmd = container.arm.new ArmAGEL2();
		cmd.addRequirements(container.arm);
		return cmd;
	}

	public Command ageL3Arm() {
		Command cmd = container.arm.new ArmAGEL3();
		cmd.addRequirements(container.arm);
		return cmd;
	}

	public Command intake() {
		Command cmd =  container.arm.new ArmToIntake()
		.andThen(container.intake.new IntakeCoral());
		cmd.addRequirements(container.arm, container.intake);
		return cmd;
	}

	public Command lower() {
		Command cmd = container.arm.new ArmToIntake();
		cmd.addRequirements(container.arm);
		return cmd;
	}

	public Command remove(DriveTrain.ScoringPosition pose) {
		Command cmd = setSuperState(State.PATHFINDING)
		.andThen(new ConditionalCommand(
			container.drivetrain.new Navigate(pose)
			.andThen(container.arm.new ArmAGEL3())
			.andThen(container.intake.new RunIntake(Seconds.of(1.2), -1, -1, -1)), 
			container.arm.new ArmAGEL2()
			.andThen(container.drivetrain.new Navigate(pose))
			.andThen(container.intake.new RunIntake(Seconds.of(1.2), 1, 1, 1)), 
		() -> {return !Constants.nearestAlgaeTagIsL2(container.drivetrain.getState().Pose);}))
		.andThen(new WaitCommand(1))
		.andThen(container.arm.new ArmToFloor())
		.andThen(setSuperState(State.TELEOP));
		cmd.addRequirements(container.arm, container.intake);
		return cmd;
	}

	public Command drive(CommandXboxController joystick) {
		return container.drivetrain.new Drive(joystick);
	}
}
