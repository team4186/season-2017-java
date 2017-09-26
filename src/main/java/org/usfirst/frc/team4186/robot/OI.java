package org.usfirst.frc.team4186.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    public final Joystick joystick = new Joystick(0);

    public final JoystickButton halfIndexTrigger = new JoystickButton(joystick, 1);
    public final JoystickButton fullIndexTrigger = new JoystickButton(joystick, 15);
    public final JoystickButton pinkyTrigger = new JoystickButton(joystick, 6);
    public final JoystickButton fire = new JoystickButton(joystick, 2);
    public final JoystickButton buttonA = new JoystickButton(joystick, 3);
    public final JoystickButton buttonB = new JoystickButton(joystick, 4);
    public final JoystickButton buttonC = new JoystickButton(joystick, 5);
    public final JoystickButton t1 = new JoystickButton(joystick, 9);
    public final JoystickButton t2 = new JoystickButton(joystick, 10);
    public final JoystickButton t3 = new JoystickButton(joystick, 11);
    public final JoystickButton t4 = new JoystickButton(joystick, 12);
    public final JoystickButton t5 = new JoystickButton(joystick, 13);
    public final JoystickButton t6 = new JoystickButton(joystick, 14);
    public final JoystickButton modeRed = new JoystickButton(joystick, 28);
    public final JoystickButton modePurple = new JoystickButton(joystick, 29);
    public final JoystickButton modeBlue = new JoystickButton(joystick, 30);
    public final JoystickButton dPadUp = new JoystickButton(joystick, 20);
    public final JoystickButton dPadRight = new JoystickButton(joystick, 21);
    public final JoystickButton dPadDown = new JoystickButton(joystick, 22);
    public final JoystickButton dPadLeft = new JoystickButton(joystick, 23);


    public OI() {
        joystick.setAxisChannel(AxisType.kThrottle, 2);
        joystick.setAxisChannel(AxisType.kTwist, 5);
    }
}

