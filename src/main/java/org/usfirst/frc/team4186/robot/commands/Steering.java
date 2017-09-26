package org.usfirst.frc.team4186.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team4186.robot.subsystems.DriveTrain;

public class Steering extends Command {

    private Joystick joystick;
    private DriveTrain driveTrain;

    public Steering(DriveTrain driveTrain, Joystick joystick) {
        super("Steering");
        requires(driveTrain);

        this.driveTrain = driveTrain;
        this.joystick = joystick;
    }

    protected void execute() {
        driveTrain.arcadeDrive(joystick.getY(), joystick.getTwist());
    }

    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
        driveTrain.stop();
    }
}
