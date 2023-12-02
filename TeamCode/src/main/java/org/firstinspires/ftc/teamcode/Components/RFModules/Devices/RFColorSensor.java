package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.apache.commons.math3.stat.StatUtils.min;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Harry
 * Class to contain all RFColorSensor functions
 */
public class RFColorSensor {
    NormalizedColorSensor colorSensor;
    RevColorSensorV3 distSensor;
    private double white = 156, purple = 214, green = 127, yellow = 81;
    private float HSV[] = {0,0,0};

    /**
     * constructor for rfcolorsensor, logs to general with CONFIG severity
     * @param p_deviceName
     */
    public RFColorSensor(String p_deviceName){
        colorSensor = op.hardwareMap.get(NormalizedColorSensor.class, p_deviceName);
        distSensor = op.hardwareMap.get(RevColorSensorV3.class, p_deviceName);
    }

    public float[] getHSV(){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), HSV);
        return HSV;
    }

    public double getDist(){
        return distSensor.getDistance(DistanceUnit.INCH);
    }

    public String getColor(){
        double errorW = 0, errorP = 0, errorG = 0, errorY = 0;
        float[] hsv = getHSV();
        errorW = Math.abs(hsv[0]-white); errorP = Math.abs(hsv[0]-purple); errorG = Math.abs(hsv[0]-green); errorY = Math.abs(hsv[0]-yellow);
        if(Math.min(Math.min(errorW, errorP),Math.min(errorY, errorG)) == errorW){
            return "WHITE";
        }
        if(Math.min(Math.min(errorW, errorP),Math.min(errorY, errorG)) == errorP){
            return "PURPLE";
        }
        if(Math.min(Math.min(errorW, errorP),Math.min(errorY, errorG)) == errorY){
            return "YELLOW";
        }
        if(Math.min(Math.min(errorW, errorP),Math.min(errorY, errorG)) == errorG){
            return "GREEN";
        }
        return "WHITE";
    }
}