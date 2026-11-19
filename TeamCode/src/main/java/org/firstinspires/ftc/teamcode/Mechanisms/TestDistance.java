package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class TestDistance {

    private DistanceSensor distance;

    public void init(HardwareMap hwMap){
        distance = hwMap.get(DistanceSensor.class, "sensor_distance");

    }
    public double getDistance(){
        return distance.getDistance(DistanceUnit.CM);
    }
}
