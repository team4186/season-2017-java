package org.usfirst.frc.team4186.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team4186.robot.subsystems.DriveTrain;

public class Steering extends Command {

    private Joystick joystick;
    private DriveTrain driveTrain;

    public Steering(DriveTrain driveTrain, Joystick joystick) {
        super("Steering");
        setInterruptible(true);
        requires(driveTrain);

        this.driveTrain = driveTrain;
        this.joystick = joystick;
    }

    protected void execute() {
        final double throttle = joystick.getY();
        final double yaw = joystick.getTwist();

        driveTrain.arcadeDrive(throttle, yaw);
    }

    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
        driveTrain.stop();
    }
}
