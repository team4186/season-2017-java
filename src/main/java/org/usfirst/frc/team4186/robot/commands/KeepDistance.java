package org.usfirst.frc.team4186.robot.commands;

import org.usfirst.frc.team4186.robot.subsystems.DistanceEstimator;
import org.usfirst.frc.team4186.robot.subsystems.DriveTrain;
import org.usfirst.frc.team4186.robot.subsystems.MotionDetector;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Command;

public class KeepDistance extends Command {
    private final DriveTrain driveTrain;
    private final PIDController pid;
    private final MotionDetector motionDetector;
    private final double distance;// in meters

    private double power = 0.0;

    public KeepDistance(DriveTrain driveTrain, DistanceEstimator distanceEstimator, MotionDetector motionDetector, double distance) {
        super("KeepDistance " + distance);
        requires(driveTrain);
        requires(distanceEstimator);
        this.driveTrain = driveTrain;
        this.motionDetector = motionDetector;
        this.distance = Math.max(Math.min(3, distance), 0.05);

        // TODO need to tune this K's
        pid = new PIDController(0.1, 0.0, 0.0, distanceEstimator, new PIDOutput() {
            @Override
            public void pidWrite(double output) {
                power = -output;
            }
        });

        pid.setInputRange(0, 3);
        pid.setAbsoluteTolerance(0.01);
        pid.setOutputRange(-0.5, 0.5);
        pid.setContinuous(false);
        pid.disable();
    }

    @Override
    protected void initialize() {
        power = 0.0;
        pid.setSetpoint(distance);
        pid.enable();
    }

    @Override
    protected void end() {
        pid.disable();
        driveTrain.stop();
    }

    @Override
    protected void execute() {
        driveTrain.tankDrive(power, power);
    }

    @Override
    protected boolean isFinished() {
        return pid.onTarget() && !motionDetector.isMoving();
    }
};