package org.usfirst.frc.team4186.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team4186.robot.subsystems.DistanceEstimator;
import org.usfirst.frc.team4186.robot.subsystems.DriveTrain;

public class KeepDistance extends Command {
    private final DriveTrain driveTrain;
    private final PIDController pid;
    private final double distance;// in meters

    public KeepDistance(DriveTrain driveTrain, DistanceEstimator distanceEstimator, double distance) {
        super("KeepDistance");
        requires(driveTrain);
        requires(distanceEstimator);
        this.driveTrain = driveTrain;
        this.distance = Math.max(Math.min(3, distance), 0.05);

        pid = new PIDController(0.03, 0, 0, distanceEstimator, new PIDOutput() {
            @Override
            public void pidWrite(double output) {
                driveTrain.tankDrive(output, output);
            }
        });

        pid.setInputRange(0, 3);
        pid.setAbsoluteTolerance(0.1);
        pid.setOutputRange(-1, 1);
        pid.setContinuous(false);
        pid.disable();
    }

    @Override
    protected void initialize() {
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
    }

    @Override
    protected boolean isFinished() {
        return pid.onTarget();
    }
};