package org.firstinspires.ftc.teamcode.Classes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// initializes a new sensor, whether it be distance or color
public class SensorClass {

    DistanceSensor Distance;
    ColorSensor Color;

    String sensor = null;

    HardwareMap hardwareMap;

    //constructor
    public SensorClass(String sensorName) {

        sensor = sensorName;

    }


    //initialize
    public void initializeDistance() {

        Distance = hardwareMap.get(DistanceSensor.class, sensor);

    }

    public void initializeColor() {

        ColorSensor Color = hardwareMap.get(ColorSensor.class, sensor);

    }


    // get values
    public void getDistanceValue() {

        telemetry.addData("deviceName",Distance.getDeviceName() );
        telemetry.addData("range", Distance.getDistance(DistanceUnit.INCH));
        telemetry.update();

    }

    public void getColorValue() {

        telemetry.addData("deviceName", Color.getDeviceName() );
        telemetry.addData("range", Color.alpha());
        telemetry.update();

    }

}
