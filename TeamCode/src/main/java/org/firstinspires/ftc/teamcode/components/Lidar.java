package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Creates an LiDAR system that can detect the distance of the nearest object to it.
 */
public class Lidar {

    DistanceSensor lidar;

    /**
     * Creates a new LiDAR object given the hardware
     */
    public Lidar(DistanceSensor distanceSensor) {
        this.lidar = distanceSensor;
    }

    /** Gets the distance of the nearest object to it.
     * @param distanceUnit the measurement we want the distance in
     * @return the distance of the nearest object in terms of distanceUnit
     */
    public double getDistance(DistanceUnit distanceUnit) {
        return lidar.getDistance(distanceUnit);
    }
}