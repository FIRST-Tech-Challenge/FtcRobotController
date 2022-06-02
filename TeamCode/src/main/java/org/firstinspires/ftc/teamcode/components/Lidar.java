package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Lidar {

    DistanceSensor lidar;
    public Lidar(DistanceSensor distanceSensor) {
        this.lidar = distanceSensor;
    }

    public double getDistance(DistanceUnit distanceUnit) {
        return lidar.getDistance(distanceUnit);
    }
}