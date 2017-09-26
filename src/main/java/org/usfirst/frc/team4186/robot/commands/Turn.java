package org.usfirst.frc.team4186.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team4186.robot.subsystems.Compass;
import org.usfirst.frc.team4186.robot.subsystems.DriveTrain;

public class Turn extends Command {

    private final DriveTrain driveTrain;
    private final PIDController pid;

    private Compass compass;
    private double angle;

    public Turn(DriveTrain driveTrain, Compass compass, double angle) {
        super("Turn " + angle);
        this.compass = compass;

        requires(driveTrain);
        this.driveTrain = driveTrain;
        this.angle = angle;

        pid = new PIDController(0.03, 0, 0, compass, new PIDOutput() {
            @Override
            public void pidWrite(double output) {
                driveTrain.tankDrive(output, -output);
            }
        });

        pid.setInputRange(-180, 180);
        pid.setAbsoluteTolerance(2);
        pid.setOutputRange(-1, 1);
        pid.setContinuous(true);
        pid.disable();
    }

    @Override
    protected void initialize() {
        compass.reset();
        pid.setSetpoint(angle);
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
