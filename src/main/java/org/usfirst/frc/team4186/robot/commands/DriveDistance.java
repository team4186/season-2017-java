package org.usfirst.frc.team4186.robot.commands;

import org.usfirst.frc.team4186.robot.subsystems.DriveTrain;
import org.usfirst.frc.team4186.robot.subsystems.MotionDetector;

import edu.wpi.first.wpilibj.command.Command;

public class DriveDistance extends Command{
	public static int auto_dist_to_time(double distance){
		
		double m = 224.4897573;
		double b = 36.143555;
		
		int time = (int)((distance*1.02 - b)*1000/m);
		return time;
	}
	
	private final long duration; // in milliseconds
    private final double power;
    private DriveTrain driveTrain;

    private long end;// in meters
	private MotionDetector motionDetector;

    public DriveDistance(DriveTrain driveTrain, MotionDetector motionDetector, double distance, double power) {
        super(String.format("Drive For %.02f meters at %.02f power", distance, power));

        requires(driveTrain);
        this.driveTrain = driveTrain;
        this.motionDetector = motionDetector;
        this.duration = auto_dist_to_time(distance);
        this.power = power;
    }

    @Override
    protected boolean isFinished() {
        return System.currentTimeMillis() > end;// && !motionDetector.isMoving();
    }

    @Override
    protected void execute() {
    	final double output = System.currentTimeMillis() > end ? 0 : power;
        driveTrain.tankDrive(output, output);
    }

    @Override
    protected void initialize() {
        end = System.currentTimeMillis() + duration;
    }

    @Override
    protected void end() {
    	driveTrain.stop();
    }
}
