package org.darbots.darbotsftclib.libcore.templates.sensors;

import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;

public abstract class DarbotsDistanceSensor implements RobotNonBlockingDevice {
    public static final double DISTANCE_INVALID = -1;
    public double ActualDistanceFactor = 1.0;

    public abstract double getDistanceInCM();
    @Override
    public boolean isBusy() {
        return false;
    }
    @Override
    public void waitUntilFinish() {
        return;
    }
}
