package org.usfirst.frc.team4186.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.Ultrasonic.Unit;
import edu.wpi.first.wpilibj.command.Subsystem;

public class DistanceEstimator extends Subsystem implements PIDSource {

	private final AnalogInput sonar;
	private final Ultrasonic shortRangeSonar;

	public DistanceEstimator(AnalogInput sonar, Ultrasonic shortRangeSonar) {
		this.sonar = sonar;
		this.shortRangeSonar = shortRangeSonar;
	}

	public void setup() {
		// TODO sonar must arrive ready to use
		sonar.setOversampleBits(4);
		sonar.setAverageBits(2);
		shortRangeSonar.setPIDSourceType(sonar.getPIDSourceType());
		shortRangeSonar.setDistanceUnits(Unit.kMillimeters);		
		shortRangeSonar.setAutomaticMode(true);
	}
	
	@Override
	public void initDefaultCommand() {
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		sonar.setPIDSourceType(pidSource);
		shortRangeSonar.setPIDSourceType(pidSource);
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return sonar.getPIDSourceType();
	}

	@Override
	public double pidGet() {
		return sonar.pidGet() < 0.30 ? shortRangeSonar.getRangeMM() / 1000.0 : sonar.pidGet();
	}
}
