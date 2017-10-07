package org.usfirst.frc.team4186.robot.subsystems;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;

public class DriveTrain extends Subsystem {
    private final RobotDrive drive;

    public DriveTrain(SpeedController left, SpeedController right) {
        drive = new RobotDrive(left, right);
        drive.setSafetyEnabled(false);
    }

    public void initDefaultCommand() {
    }

    public void arcadeDrive(double moveValue, double rotateValue) {
        drive.arcadeDrive(inertiaPowerCorrection(moveValue), inertiaPowerCorrection(rotateValue));
    }

    public void tankDrive(double left, double right) {
        drive.tankDrive(left, right, false);
    }

    public void stop() {
        drive.stopMotor();
    }

    private double inertiaPowerCorrection(double power) {
        final double deadzone = 0.3;
        return (power * (1 - deadzone)) + Math.copySign(deadzone, power);
    }
}
