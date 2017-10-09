package org.usfirst.frc.team4186.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.tables.ITable;

public class MotionDetector extends Subsystem {
    private final AHRS ahrs;

    public MotionDetector(AHRS ahrs) {
        this.ahrs = ahrs;
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    public double getVelocityX() {
        return ahrs.getVelocityX();
    }

    public double getVelocityY() {
        return ahrs.getVelocityY();
    }

    public double getVelocityZ() {
        return ahrs.getVelocityZ();
    }

    public double getSpeed() {
        return Math.sqrt(
                ahrs.getVelocityX() * ahrs.getVelocityX() +
                        ahrs.getVelocityY() * ahrs.getVelocityY() +
                        ahrs.getVelocityZ() * ahrs.getVelocityZ()
        );
    }

    public double getAccelerationX() {
        return ahrs.getWorldLinearAccelX();
    }

    public double getAccelerationY() {
        return ahrs.getWorldLinearAccelY();
    }

    public double getAccelerationZ() {
        return ahrs.getWorldLinearAccelZ();
    }

    public double getAccelerationMagnitude() {
        return Math.sqrt(
                ahrs.getWorldLinearAccelX() * ahrs.getWorldLinearAccelX() +
                        ahrs.getWorldLinearAccelY() * ahrs.getWorldLinearAccelY() +
                        ahrs.getWorldLinearAccelZ() * ahrs.getWorldLinearAccelZ()
        );
    }

    public boolean isMoving() {
        return ahrs.isMoving();
    }

    @Override
    public void initTable(ITable table) {
        super.initTable(table);
        table.putBoolean("moving", isMoving());
    }
}