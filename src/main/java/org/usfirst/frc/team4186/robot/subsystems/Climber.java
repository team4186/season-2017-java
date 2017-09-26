package org.usfirst.frc.team4186.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Climber extends Subsystem {
	private final SpeedController motor;
    
	public Climber(SpeedController motor) {
		super("Climber");
		this.motor = motor;
	}
	
	public void stop() {
		motor.stopMotor();
	}
	
	public void climbUp() {
		motor.set(1);
	}
	
	public void climbDown() {
		motor.set(-1);
	}
	
	@Override
	protected void initDefaultCommand() {
	}
}

