package org.darbots.darbotsftclib.libcore.sensors.gyros;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.templates.other_sensors.RobotGyro;

public class SynchronizedSoftwareGyro implements RobotGyro {
    protected Float m_LastAng = new Float(0.0f);

    public SynchronizedSoftwareGyro(){
        this.m_LastAng = 0f;
    }

    public SynchronizedSoftwareGyro(float initialAngle){
        this.m_LastAng = initialAngle;
    }

    public SynchronizedSoftwareGyro(RobotGyro oldGyro){
        this.m_LastAng = oldGyro.getHeading();
    }

    @Override
    public float getHeading() {
        synchronized (m_LastAng) {
            return m_LastAng.floatValue();
        }
    }

    @Override
    public HeadingRotationPositiveOrientation getHeadingRotationPositiveOrientation() {
        return HeadingRotationPositiveOrientation.CounterClockwise;
    }

    public void setHeading(float heading){
        synchronized (m_LastAng) {
            this.m_LastAng = XYPlaneCalculations.normalizeDeg(heading);
        }
    }

    public void offsetHeading(float deltaAng){
        synchronized (m_LastAng) {
            this.m_LastAng = XYPlaneCalculations.normalizeDeg(m_LastAng + deltaAng);
        }
    }
}
