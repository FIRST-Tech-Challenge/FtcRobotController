package org.firstinspires.ftc.teampractice.examples;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teampractice.IObjectDetector;

public class DistanceDetector implements IObjectDetector<Boolean> {
    private DistanceSensor sensorRange;
    private double threshold = 2;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean detected = false;
    private boolean away = false;

    /**
     * Set distance detector
     *
     * @param sensorToUse
     * @param threshold   in CM
     * @param away        leaving or arriving
     */
    public DistanceDetector(DistanceSensor sensorToUse, double threshold, boolean away) {
        sensorRange = sensorToUse;
        this.threshold = threshold;
        this.away = away;
    }

    /**
     * Set distance detector
     *
     * @param sensorToUse
     * @param threshold   in CM
     */
    public DistanceDetector(DistanceSensor sensorToUse, double threshold) {
        this(sensorToUse, threshold, false);
    }

    @Override
    public Boolean objectDetected() {
        double currentDistance = sensorRange.getDistance(DistanceUnit.CM);
        detected = false;

        if ((away && currentDistance > threshold) || (!away && currentDistance < threshold)) {
            //if the detection last for 3 millisesoncds then sounds like real
            if (runtime.milliseconds() > 3d) {
                detected = true;
            }
        } else {
            runtime.reset();
        }

        return detected;
    }

    public double getCurrentDistance() {
        return sensorRange.getDistance(DistanceUnit.CM);
    }

    public void setThreshold(double threshold) {
        this.threshold = threshold;
        reset();
    }

    public void reset() {
        runtime.reset();
        detected = false;
    }
}
