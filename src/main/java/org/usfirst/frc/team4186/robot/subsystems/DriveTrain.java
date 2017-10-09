package org.usfirst.frc.team4186.robot.subsystems;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain extends Subsystem {

	private final RobotDrive drive;
	private double direction = 1.0;

	public DriveTrain(SpeedController left, SpeedController right) {
		drive = new RobotDrive(left, right);
		drive.setSafetyEnabled(false);
	}

	public void initDefaultCommand() {
	}

	public void arcadeDrive(double moveValue, double rotateValue) {
		final double move_mapped = choose_map(moveValue, 2);
		final double rotate_mapped = choose_map(rotateValue, 2);
		SmartDashboard.putNumber("move_mapped", move_mapped);
		SmartDashboard.putBoolean("drive direction", direction > 0);

		drive.arcadeDrive(move_mapped * direction, rotate_mapped, false);
	}

	public void tankDrive(double left, double right) {
		System.out.printf("Tank Drive l:%.03f r:%.3f\n", left, right);
		drive.tankDrive(inertiaPowerCorrection(left), inertiaPowerCorrection(right), false);
	}

	public void stop() {
		drive.stopMotor();
	}

	private double inertiaPowerCorrection(double power) {
		final double deadzone = 0.1;
		return (power * (1 - deadzone)) + Math.copySign(deadzone, power);
	}

	private double joy_map_exponential(double joy_y) {
		double k = 0.1;
		double l = 0.05;

		double output = Math.signum(joy_y) * Math.pow(k, (Math.abs(joy_y) - 1) / (l - 1));
		return output;
	}

	private double joy_map_circle_segment(double joy_y) {
		double k = 0.1;
		double l = 0.05;

		double output = Math.signum(joy_y)
				* ((k - 1) * Math.sqrt(Math.pow((1 - l), 2) - Math.pow((Math.abs(joy_y) - l), 2)) / (1 - l) + 1);
		return output;
	}

	private double joy_map_cos(double joy_y) {
		double k = 0.1;
		double l = 0.05;

		double output = Math.signum(joy_y)
				* ((k - 1) / (Math.cos(Math.PI * l + Math.PI) - 1) * Math.cos(Math.PI * Math.abs(joy_y) + Math.PI)
						+ (1 - (k - 1) / (Math.cos(Math.PI * l + Math.PI) - 1)));
		return output;
	}

	private double joy_map_logistic(double joy_y) {
		double k = 0.1;
		double l = 0.05;
		double a = (k - 1) / (1 / (1 + Math.exp(4)) + 1 / (1 + Math.exp(10 * (l - 0.6))));

		double output = Math.signum(joy_y)
				* (-a / (1 + Math.exp(10 * (Math.abs(joy_y) - 0.6)) + 1 + a / (1 + Math.exp(5))));

		return output;
	}

	private double choose_map(double joy_y, int function_index) {
		switch (function_index) {
		case 0:
			return joy_y;
		case 1:
			return joy_map_exponential(joy_y);
		case 2:
			return joy_map_circle_segment(joy_y);
		case 3:
			return joy_map_cos(joy_y);
		case 4:
			return joy_map_logistic(joy_y);
		}
		return joy_y;
	}

	public void switch_mode() {
		direction *= -1.0;

	}
}
