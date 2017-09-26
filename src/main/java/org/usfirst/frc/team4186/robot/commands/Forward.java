package org.usfirst.frc.team4186.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team4186.robot.subsystems.DriveTrain;
import org.usfirst.frc.team4186.robot.subsystems.Odometer;

public class Forward extends Command {

    private final DriveTrain driveTrain;
    private final PIDController pid;
    private final Odometer odometer;
    private final double distance;// in meters

    public Forward(DriveTrain driveTrain, Odometer odometer, double distance) {
        super("Forward " + distance);
        this.odometer = odometer;

        requires(driveTrain);
        this.driveTrain = driveTrain;
        this.distance = distance;

        pid = new PIDController(0.03, 0, 0, odometer, new PIDOutput() {
            @Override
            public void pidWrite(double output) {
                driveTrain.arcadeDrive(output, output);
            }
        });

        pid.setInputRange(Double.MIN_VALUE, Double.MAX_VALUE);
        pid.setAbsoluteTolerance(0.1);
        pid.setOutputRange(-1, 1);
        pid.setContinuous(false);
        pid.disable();
    }

    @Override
    protected void initialize() {
        odometer.reset();
        pid.setSetpoint(distance);
        pid.enable();
    }

    @Override
    protected boolean isFinished() {
        return pid.onTarget();
    }

    @Override
    protected void execute() {
    }

    @Override
    protected void end() {
        pid.disable();
        driveTrain.stop();
    }
}
