package org.usfirst.frc.team4186.robot;

import org.usfirst.frc.team4186.robot.commands.ClimbDown;
import org.usfirst.frc.team4186.robot.commands.ClimbUp;
import org.usfirst.frc.team4186.robot.commands.DriveDistance;
import org.usfirst.frc.team4186.robot.commands.KeepDistance;
import org.usfirst.frc.team4186.robot.commands.Steering;
import org.usfirst.frc.team4186.robot.commands.Turn;
import org.usfirst.frc.team4186.robot.factories.CANConfigFactory;
import org.usfirst.frc.team4186.robot.factories.MotorFactory;
import org.usfirst.frc.team4186.robot.subsystems.Climber;
import org.usfirst.frc.team4186.robot.subsystems.Compass;
import org.usfirst.frc.team4186.robot.subsystems.DistanceEstimator;
import org.usfirst.frc.team4186.robot.subsystems.DriveTrain;
import org.usfirst.frc.team4186.robot.subsystems.MotionDetector;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.PrintCommand;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

	private OI oi = new OI();

	// Hardware
	private MotorFactory motors = new CANConfigFactory();
	private AHRS ahrs = new AHRS(SPI.Port.kMXP);
	private AnalogInput sonar = new AnalogInput(RobotMap.SENSORS.SONAR_0);
	private Ultrasonic shortRangeSonar = new Ultrasonic(1, 0);

	private SpeedController leftMotor = motors.leftMotor();
	private SpeedController rightMotor = motors.rightMotor();
	private SpeedController climberMotor = motors.climberMotor();

	// Sub systems
	//// Actuators
	private DriveTrain driveTrain = new DriveTrain(leftMotor, rightMotor);
	private Climber climber = new Climber(climberMotor);
	//// Sensors
	private Compass compass = new Compass(ahrs);
	private MotionDetector motionDetector = new MotionDetector(ahrs);
	private DistanceEstimator distanceEstimator = new DistanceEstimator(sonar, shortRangeSonar);

	// Commands
	private ClimbUp climbUp = new ClimbUp(climber);
	private ClimbDown climbDown = new ClimbDown(climber);
	private Steering steering = new Steering(driveTrain, oi.joystick);

	// Autonomous
	private Command autonomousCommand = new PrintCommand("I'm an empty Autonomous!");
	private SendableChooser<Command> chooser;

	public void robotInit() {

		CameraServer.getInstance().startAutomaticCapture(0);
		CameraServer.getInstance().startAutomaticCapture(1);

		oi = new OI();

		chooser = new SendableChooser<>();
		chooser.addDefault("Go Straight", autonomousGoForward());
		chooser.addObject("Turn Left", autonomousTurnLeft());
		chooser.addObject("Turn Right", autonomousTurnRight());
		SmartDashboard.putData("Auto mode", chooser);

		distanceEstimator.setup();
	}

	public void disabledInit() {
		autonomousCommand.cancel();
		climbUp.cancel();
		climbDown.cancel();
		steering.cancel();
	}

	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	public void autonomousInit() {
		autonomousCommand = getSelectedAutonomous();
		autonomousCommand.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	public void teleopInit() {
		oi.dPadUp.whenActive(climbUp);
		oi.dPadUp.whenInactive(new InstantCommand() {
			@Override
			protected void execute() {
				climbUp.cancel();
			}

		});

		oi.dPadDown.whenActive(climbDown);
		oi.dPadDown.whenInactive(new InstantCommand() {
			@Override
			protected void execute() {
				climbDown.cancel();
			}
		});

		Command kd = new DriveDistance(driveTrain, motionDetector, 250.0, 0.4);//new KeepDistance(driveTrain, distanceEstimator, motionDetector, 1.0);
		oi.fire.whenActive(kd);
		oi.fire.whenInactive(new InstantCommand() {
			@Override
			protected void execute() {
				kd.cancel();
			}

			@Override
			protected void end() {
				steering.start();
			}
		});

		steering.start();

		{
			Turn turn = new Turn(driveTrain, compass, motionDetector, 90);

			oi.t1.whenActive(turn);
			oi.t1.whenInactive(new InstantCommand() {
				@Override
				protected void execute() {
					turn.cancel();
				}

				@Override
				protected void end() {
					steering.start();
				}
			});
		}
		{
			Turn turn = new Turn(driveTrain, compass, motionDetector, -90);
			oi.t2.whenActive(turn);
			oi.t2.whenInactive(new InstantCommand() {
				@Override
				protected void execute() {
					turn.cancel();
				}

				@Override
				protected void end() {
					steering.start();
				}
			});
		}

		oi.buttonB.whenPressed(new InstantCommand() {
			@Override
			protected void execute() {
				driveTrain.switch_mode();
			}
		});
	}

	public void teleopPeriodic() {
		updateDashboard();
		Scheduler.getInstance().run();
	}

	public void testPeriodic() {
		LiveWindow.run();
	}

	private Command getSelectedAutonomous() {
		Command selected = chooser.getSelected();
		return selected != null ? selected : new PrintCommand("I'm an empty Autonomous!");
	}

	private Command autonomousGoForward() {
		return new KeepDistance(driveTrain, distanceEstimator, motionDetector, 0.1);
	}

	private Command autonomousTurnLeft() {
		return autonomousTurn(-120);
	}

	private Command autonomousTurnRight() {
		return autonomousTurn(120);
	}

	private Command autonomousTurn(double angle) {
		CommandGroup cmd = new CommandGroup();
		cmd.addSequential(new KeepDistance(driveTrain, distanceEstimator, motionDetector, 264.7));
		cmd.addSequential(new Turn(driveTrain, compass, motionDetector, angle));
		//cmd.addSequential(new KeepDistance(driveTrain, distanceEstimator, motionDetector, 0.1));
		return cmd;
	}

	private void updateDashboard() {
		// final double throttle = oi.joystick.getY();
		// final double yaw = oi.joystick.getTwist();

		// TODO unchecked cast can crash the robot!!
		// SmartDashboard.putNumber("LeftMotor current", ((CANTalon)
		// leftMotor).getOutputCurrent());
		// SmartDashboard.putNumber("LeftMotor voltage", ((CANTalon)
		// leftMotor).getOutputVoltage());
		//
		// SmartDashboard.putNumber("RightMotor current", ((CANTalon)
		// rightMotor).getOutputCurrent());
		// SmartDashboard.putNumber("RightMotor voltage", ((CANTalon)
		// rightMotor).getOutputVoltage());
		//
		// SmartDashboard.putNumber("Climber current", ((CANTalon)
		// climberMotor).getOutputCurrent());
		// SmartDashboard.putNumber("Climber voltage", ((CANTalon)
		// climberMotor).getOutputVoltage());
		//
		// SmartDashboard.putNumber("throttle", throttle);
		// SmartDashboard.putNumber("turn", yaw);

		SmartDashboard.putNumber("distance", distanceEstimator.pidGet());
		SmartDashboard.putNumber("distance short range", shortRangeSonar.getRangeMM() / 1000.0);
		SmartDashboard.putNumber("distance long range", sonar.pidGet());

		// SmartDashboard.putBoolean("rotating", compass.isRotating());
		// SmartDashboard.putNumber("gyro", compass.getHeading());
		// SmartDashboard.putNumber("yaw", compass.pidGet());
		//
		// SmartDashboard.putBoolean("moving", motionDetector.isMoving());
		// SmartDashboard.putNumber("velocity[X]",
		// motionDetector.getVelocityX());
		// SmartDashboard.putNumber("velocity[Y]",
		// motionDetector.getVelocityY());
		// SmartDashboard.putNumber("velocity[Z]",
		// motionDetector.getVelocityZ());
		// SmartDashboard.putNumber("velocity", motionDetector.getSpeed());
		// SmartDashboard.putNumber("acceleration[X]",
		// motionDetector.getAccelerationX());
		// SmartDashboard.putNumber("acceleration[Y]",
		// motionDetector.getAccelerationY());
		// SmartDashboard.putNumber("acceleration[Z]",
		// motionDetector.getAccelerationZ());
		// SmartDashboard.putNumber("acceleration",
		// motionDetector.getAccelerationMagnitude());
	}
}
