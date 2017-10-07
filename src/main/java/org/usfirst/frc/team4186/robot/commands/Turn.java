package org.usfirst.frc.team4186.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team4186.robot.subsystems.Compass;
import org.usfirst.frc.team4186.robot.subsystems.DriveTrain;
import org.usfirst.frc.team4186.robot.subsystems.MotionDetector;

public class Turn extends Command {

    private final DriveTrain driveTrain;
    private final PIDController pid;

    private Compass compass;
    private double angle;

    private double power = 0;
    private MotionDetector motionDetector;

    public Turn(DriveTrain driveTrain, Compass compass, MotionDetector motionDetector, double angle) {
        super("Turn " + angle);
        requires(driveTrain);
        requires(compass);

        this.compass = compass;
        this.motionDetector = motionDetector;
        this.driveTrain = driveTrain;
        this.angle = angle;

        // TODO need to tune this K's
        pid = new PIDController(0.01, 0.0, 0.0, compass, new PIDOutput() {
            @Override
            public void pidWrite(double output) {
                power = output;
            }
        });

        pid.setInputRange(-180, 180);
        pid.setAbsoluteTolerance(0.5);
        pid.setOutputRange(-0.5, 0.5);
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
        return pid.onTarget() && !motionDetector.isMoving() && !compass.isRotating();
    }

    @Override
    protected void execute() {

        double corrected = (power * 0.6) + Math.copySign(power, 0.4);
        System.out.printf("Turn=%s Power=%s Error=%.0f\n", power, corrected, pid.getError());
        driveTrain.tankDrive(-corrected, corrected);
    }

    @Override
    protected void end() {
        pid.disable();
        driveTrain.stop();
    }
}
