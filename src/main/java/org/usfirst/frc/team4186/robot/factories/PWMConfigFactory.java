package org.usfirst.frc.team4186.robot.factories;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import org.usfirst.frc.team4186.robot.RobotMap;

public class PWMConfigFactory implements MotorFactory {

    @Override
    public SpeedController intakeMotor() {
        return new Talon(RobotMap.PWM.INTAKE);
    }

    @Override
    public SpeedController shooterMotor() {
        return new Talon(RobotMap.PWM.SHOOTER);
    }

    @Override
    public SpeedController feederMotor() {
        return new Spark(RobotMap.PWM.FEEDER);
    }

    @Override
    public SpeedController leftMotor() {
        SpeedController l = new Talon(RobotMap.PWM.LEFT);
        l.setInverted(true);
        return l;
    }

    @Override
    public SpeedController rightMotor() {
        SpeedController r = new Talon(RobotMap.PWM.RIGHT);
        r.setInverted(true);
        return r;
    }

    @Override
    public SpeedController climberMotor() {
        return new Talon(RobotMap.PWM.CLIMBER);
    }

    @Override
    public Servo hopperRotationMotor() {
        return new Servo(RobotMap.PWM.HOPPER_ROTATION);
    }

}