package org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Misc;
/*
    Configured this as per the diagram on below link except ReV Color SensorV3
    was named as sensor_color_distance.
    https://docs.revrobotics.com/color-sensor/application-examples

    Basically connect to any I2C bus and configure port 0 to nothing or expansion hub
    and port 1 to Rev Color Sensor V3 and configure as sensor_color_distance
    One problem encountered after configuring I2C sensor was still problem after saving
    configuration. So had to do SCAN on configuration for it to make this work

 */
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorDistanceRevV3
{

    // Define class members
    HardwareMap  hwMap      =  null;
    /* local OpMode members. */
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;


    public ColorDistanceRevV3() {
        // Save reference to Hardware map
        hwMap = op.hardwareMap;
        // get a reference to the color sensor.
        sensorColor = hwMap.get(ColorSensor.class, "intakeDistSensor");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hwMap.get(DistanceSensor.class, "intakeDistSensor");
    }

    public int alpha() {
        return sensorColor.alpha();
    }
    public int red() {
        return sensorColor.red();
    }
    public int green() {
        return sensorColor.green();
    }
    public int blue() {
        return sensorColor.blue();
    }
    public float [] hsvVal() {
        float hsvValues[] = {0F, 0F, 0F};
        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);
        return hsvValues;
    }
    public boolean isBall(){
        if(red()/(float)blue()>1.0){
            return false;
        }
        else{
            return true;
        }
    }

    // unit can have one of following values
    //       DistanceUnit.MM
    //       DistanceUnit.CM
    //       DistanceUnit.METER
    //       DistanceUnit.INCH
    public double getSensorDistance() {
        return sensorDistance.getDistance(DistanceUnit.INCH);
    }
}

