package org.usfirst.frc.team4186.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team4186.robot.subsystems.DriveTrain;

public class DriveTime extends Command {
    private final long duration; // in milliseconds
    private final double power;
    private DriveTrain driveTrain;

    private long end;// in meters

    public DriveTime(DriveTrain driveTrain, long duration, double power) {
        super("DriveTime");

        requires(driveTrain);
        this.driveTrain = driveTrain;
        this.duration = duration;
        this.power = power;
    }

    @Override
    protected boolean isFinished() {
        return System.currentTimeMillis() > end;
    }

    @Override
    protected void execute() {
        driveTrain.tankDrive(power, power);
    }

    @Override
    protected void initialize() {
        end = System.currentTimeMillis() + duration;
    }

    @Override
    protected void end() {
    }
};