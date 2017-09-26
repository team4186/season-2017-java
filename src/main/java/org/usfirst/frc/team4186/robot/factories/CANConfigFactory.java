package org.usfirst.frc.team4186.robot.factories;

import com.ctre.MotorControl.CANTalon;
import com.ctre.MotorControl.SmartMotorController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SpeedController;
import org.usfirst.frc.team4186.robot.RobotMap;

public class CANConfigFactory implements MotorFactory {

    @Override
    public SpeedController shooterMotor() {
        return new CANTalon(RobotMap.CAN.SHOOTER);
    }

    @Override
    public SpeedController intakeMotor() {
        return new CANTalon(RobotMap.CAN.SHOOTER);
    }

    @Override
    public Servo hopperRotationMotor() {
        return new Servo(RobotMap.CAN.HOPPER_ROTATION);
    }

    @Override
    public SpeedController feederMotor() {
        return new CANTalon(RobotMap.CAN.FEEDER);
    }

    @Override
    public SpeedController leftMotor() {
        return mergeMotors(RobotMap.CAN.LEFTF, RobotMap.CAN.LEFTB);
    }

    @Override
    public SpeedController rightMotor() {
        return mergeMotors(RobotMap.CAN.RIGHTF, RobotMap.CAN.RIGHTB);
    }

    @Override
    public SpeedController climberMotor() {
        return mergeMotors(RobotMap.CAN.CLIMBERF, RobotMap.CAN.CLIMBERB);
    }

    private static SpeedController mergeMotors(int... motorIds) {
        CANTalon[] motors = new CANTalon[motorIds.length];
        final int masterId = motorIds[0];
        motors[0] = new CANTalon(motorIds[0]);
        motors[0].changeControlMode(SmartMotorController.TalonControlMode.PercentVbus);
        for (int i = 1; i < motors.length; ++i) {
            motors[i] = new CANTalon(motorIds[i]);
            motors[i].changeControlMode(SmartMotorController.TalonControlMode.Follower);
            motors[i].set(masterId);
        }
        return motors[0];
    }
}