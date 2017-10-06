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
        drive.arcadeDrive(moveValue, rotateValue);
    }

    public void tankDrive(double left, double right) {
        drive.tankDrive(left, right);
    }

    public void drive(double power, double curve) {
        drive.drive(power, curve);
    }

    public void stop() {
        drive.stopMotor();
    }
}
