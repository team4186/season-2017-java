package org.usfirst.frc.team4186.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Compass extends Subsystem implements PIDSource {
	private final AHRS ahrs;

	public Compass(AHRS ahrs) {
		this.ahrs = ahrs;
	}

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		ahrs.setPIDSourceType(pidSource);
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return ahrs.getPIDSourceType();
	}

	@Override
	public double pidGet() {
		return ahrs.pidGet();
	}

	public void reset() {
		ahrs.reset();
	}
}
