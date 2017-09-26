package org.usfirst.frc.team4186.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team4186.robot.subsystems.Climber;

/**
 *
 */
public class ClimbDown extends Command {

    private Climber climber;

    public ClimbDown(Climber climber) {
        requires(climber);
        this.climber = climber;
    }

    @Override
    protected void execute() {
        climber.climbDown();
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        climber.stop();
    }
}
