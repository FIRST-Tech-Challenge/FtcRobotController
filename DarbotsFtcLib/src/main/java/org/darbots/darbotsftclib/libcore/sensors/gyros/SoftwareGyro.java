package org.darbots.darbotsftclib.libcore.sensors.gyros;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.templates.other_sensors.RobotGyro;

public class SoftwareGyro implements RobotGyro {
    protected float m_LastAng = 0;

    public SoftwareGyro(){
        this.m_LastAng = 0f;
    }

    public SoftwareGyro(float initialAngle){
        this.m_LastAng = initialAngle;
    }

    public SoftwareGyro(RobotGyro oldGyro){
        this.m_LastAng = oldGyro.getHeading();
    }

    @Override
    public float getHeading() {
        return m_LastAng;
    }

    public void setHeading(float heading){
        this.m_LastAng = XYPlaneCalculations.normalizeDeg(heading);
    }

    public void offsetHeading(float deltaAng){
        this.m_LastAng = XYPlaneCalculations.normalizeDeg(m_LastAng + deltaAng);
    }

    @Override
    public HeadingRotationPositiveOrientation getHeadingRotationPositiveOrientation() {
        return HeadingRotationPositiveOrientation.CounterClockwise;
    }
}
