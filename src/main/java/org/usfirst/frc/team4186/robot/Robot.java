
package org.usfirst.frc.team4186.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4186.robot.commands.*;
import org.usfirst.frc.team4186.robot.factories.CANConfigFactory;
import org.usfirst.frc.team4186.robot.factories.MotorFactory;
import org.usfirst.frc.team4186.robot.subsystems.*;

public class Robot extends IterativeRobot {

    private OI oi = new OI();

    // Hardware
    private MotorFactory motors = new CANConfigFactory();
    private AHRS ahrs = new AHRS(SPI.Port.kMXP);
    private Encoder encoder = new Encoder(RobotMap.SENSORS.ENCODER_0, RobotMap.SENSORS.ENCODER_1);
    private AnalogInput sonar = new AnalogInput(RobotMap.SENSORS.SONAR_0);

    // Sub systems
    //// Actuators
    private DriveTrain driveTrain = new DriveTrain(motors.leftMotor(), motors.rightMotor());
    private Climber climber = new Climber(motors.climberMotor());
    //// Sensors
    private Compass compass = new Compass(ahrs);
    private Odometer odometer = new Odometer(encoder);
    private DistanceEstimator distanceEstimator = new DistanceEstimator(sonar);

    // Commands
    private ClimbUp climbUp = new ClimbUp(climber);
    private ClimbDown climbDown = new ClimbDown(climber);
    private Steering steering = new Steering(driveTrain, oi.joystick);

    // Autonomous
    private Command autonomousCommand = new PrintCommand("I'm an empty Autonomous!");
    private SendableChooser<Command> chooser;

    public void robotInit() {
        oi = new OI();

        chooser = new SendableChooser<>();
        chooser.addDefault("Go Straight", autonomousGoForward());
        chooser.addObject("Turn Left", autonomousTurnLeft());
        chooser.addObject("Turn Right", autonomousTurnRight());
        SmartDashboard.putData("Auto mode", chooser);

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
        /*
         * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

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

        steering.start();
    }

    public void teleopPeriodic() {
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
        return new KeepDistance(driveTrain, distanceEstimator, 0.0);
    }

    private Command autonomousTurnLeft() {
        return autonomousTurn(-60);
    }

    private Command autonomousTurnRight() {
        return autonomousTurn(60);
    }

    private Command autonomousTurn(double angle) {
        CommandGroup cmd = new CommandGroup();
        cmd.addSequential(new KeepDistance(driveTrain, distanceEstimator, 264.7));
        cmd.addSequential(new Turn(driveTrain, compass, angle));
        cmd.addSequential(new KeepDistance(driveTrain, distanceEstimator, 0.0));
        return cmd;
    }
}
