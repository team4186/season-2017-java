package org.usfirst.frc.team4186.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team4186.robot.subsystems.DriveTrain;
import org.usfirst.frc.team4186.robot.subsystems.MotionDetector;

public class DriveTime extends Command {
    private final long duration; // in milliseconds
    private final double power;
    private DriveTrain driveTrain;

    private long end;// in meters
	private MotionDetector motionDetector;

    public DriveTime(DriveTrain driveTrain, MotionDetector motionDetector, long duration, double power) {
        super(String.format("Drive For %.02f seconds at %.02f power", duration / 1000f, power));

        requires(driveTrain);
        this.driveTrain = driveTrain;
        this.motionDetector = motionDetector;
        this.duration = duration;
        this.power = power;
    }

    @Override
    protected boolean isFinished() {
        return System.currentTimeMillis() > end && !motionDetector.isMoving();
    }

    @Override
    protected void execute() {
        driveTrain.tankDrive(power, power);
    }

    @Override
    protected void initialize() {
        end = System.currentTimeMillis() + duration;
    }
};