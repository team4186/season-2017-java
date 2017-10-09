package org.usfirst.frc.team4186.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.tables.ITable;

public class DistanceEstimator extends Subsystem implements PIDSource {

    private final AnalogInput sonar;

    public DistanceEstimator(AnalogInput sonar) {
        this.sonar = sonar;
        // TODO sonar must arrive ready to use
        sonar.setOversampleBits(4);
        sonar.setAverageBits(2);
    }

    @Override
    public void initDefaultCommand() {
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        sonar.setPIDSourceType(pidSource);
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return sonar.getPIDSourceType();
    }

    @Override
    public double pidGet() {
        return sonar.pidGet();
    }

    @Override
    public void initTable(ITable table) {
        super.initTable(table);
        table.putNumber("distance", pidGet());
    }
}
