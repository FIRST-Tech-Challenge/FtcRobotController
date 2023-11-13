package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team15091.IObjectDetector;

public class DistanceDetector implements IObjectDetector<Boolean> {
    private DistanceSensor sensorRange;
    private double threshold = 2;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean detected = false;
    private boolean away = false;

    // Initialize variables for filtering
    private double filteredDistance = 0.0;
    private double alpha = 0.2; // Adjust this value based on your needs

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

        // Apply exponential moving average filter
        filteredDistance = alpha * currentDistance + (1 - alpha) * filteredDistance;

        detected = false;

        if ((away && filteredDistance > threshold) || (!away && filteredDistance < threshold)) {
            //if the detection last for 3 milliseconds then sounds like real
            if (runtime.milliseconds() > 3d) {
                detected = true;
            }
        } else {
            runtime.reset();
        }

        return detected;
    }

    public double getCurrentDistance() {
        double currentDistance = sensorRange.getDistance(DistanceUnit.CM);

        // Apply exponential moving average filter
        filteredDistance = alpha * currentDistance + (1 - alpha) * filteredDistance;

        return filteredDistance;
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
