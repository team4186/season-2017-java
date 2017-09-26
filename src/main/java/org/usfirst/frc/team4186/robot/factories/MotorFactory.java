package org.usfirst.frc.team4186.robot.factories;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SpeedController;

public interface MotorFactory {
    SpeedController intakeMotor();

    SpeedController shooterMotor();

    SpeedController feederMotor();

    SpeedController climberMotor();

    SpeedController leftMotor();

    SpeedController rightMotor();

    Servo hopperRotationMotor();
}
