package org.usfirst.frc.team4186.robot;

public interface RobotMap {
    interface CAN {
        int LEFTF = 1;
        int LEFTB = 2;

        int RIGHTF = 9;
        int RIGHTB = 8;

        int CLIMBERF = 6;
        int CLIMBERB = 7;

        int INTAKE = 3;
        int SHOOTER = 4;
        int FEEDER = 5;

        int HOPPER_ROTATION = 5;
    }

    interface PWM {
        int RIGHT = 0;  // NEEDS Y CABLE
        int LEFT = 1; // NEEDS Y CABLE
        int CLIMBER = 6;  // NEEDS Y CABLE
        int INTAKE = 4;
        int SHOOTER = 3;
        int HOPPER_ROTATION = 5;
        int FEEDER = 2;
    }

    interface SENSORS {
        int SONAR_0 = 0;

        int ENCODER_0 = 8;
        int ENCODER_1 = 9;
    }
}
