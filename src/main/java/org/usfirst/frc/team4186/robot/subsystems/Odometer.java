package org.usfirst.frc.team4186.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Odometer extends Subsystem implements PIDSource {
    private static final double DISTANCE_PER_PULSE = .026703;

    private final Encoder encoder;

    public Odometer(Encoder encoder) {
        this.encoder = encoder;
        // TODO encoder must arrive here already configured!
        this.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        setPIDSourceType(PIDSourceType.kDisplacement);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        encoder.setPIDSourceType(pidSource);
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return encoder.getPIDSourceType();
    }

    @Override
    public double pidGet() {
        return encoder.pidGet();
    }

    public void reset() {
        encoder.reset();
    }
}
