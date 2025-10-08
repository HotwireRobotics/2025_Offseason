package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotations;

import java.lang.reflect.Executable;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmToPose extends Command {

	private Angle wristTarget;
	private Angle armTarget;
	private Arm arm;

	public ArmToPose(Angle wristTarget, Angle armTarget, Arm arm) {
		this.armTarget = armTarget;
		this.wristTarget = wristTarget;
		this.arm = arm;
	}

	@Override
	public void initialize() {
		arm.setArmMotorPosition(this.armTarget.magnitude());
		arm.setWristMotorPosition(this.wristTarget.magnitude());
	}

	@Override
	public void end(boolean interrupted) {
		
	}

	@Override
	public boolean isFinished() {
		return (
			arm.isArmAtPosition(armTarget, Rotations.of(0.015)) &&
			arm.isWristAtPosition(wristTarget, Rotations.of(0.015))
		);
	}
}