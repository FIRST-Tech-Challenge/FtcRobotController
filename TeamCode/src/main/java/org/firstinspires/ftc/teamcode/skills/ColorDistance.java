package org.firstinspires.ftc.teamcode.skills;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by sjeltuhin on 9/26/17.
 */

public class ColorDistance {
    ColorSensor colorSensor;
    DistanceSensor distanceSensor;
    ElapsedTime runtime = new ElapsedTime();

    public void init(HardwareMap hardwareMap){
        // Get a reference to our sensor object.
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "color_sensor");

        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
       // toggleLight(false);
    }

//    private void toggleLight(boolean on){
//        if (colorSensor instanceof SwitchableLight) {
//            ((SwitchableLight) colorSensor).enableLight(on);
//        }
//    }


    public DetectedColor detectColor(Telemetry telemetry, float timeout) {
        //toggleLight(true);
        // values is a reference to the hsvValues array.
        DetectedColor dc = DetectedColor.NONE;
        float hsvValues[] = {0F, 0F, 0F};

        final double SCALE_FACTOR = 255;

        runtime.reset();
        boolean stop = false;
        while(!stop) {
                    // Read the sensor
            Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
                    (int) (colorSensor.green() * SCALE_FACTOR),
                    (int) (colorSensor.blue() * SCALE_FACTOR),
                    hsvValues);


//            telemetry.addLine()
//                    .addData("H", "%.3f", hsvValues[0])
//                    .addData("S", "%.3f", hsvValues[1])
//                    .addData("V", "%.3f", hsvValues[2]);
            telemetry.addLine()
                    .addData("a", "%d", colorSensor.alpha())
                    .addData("r", "%d", colorSensor.red())
                    .addData("g", "%d", colorSensor.green())
                    .addData("b", "%d", colorSensor.blue())
            .addData("Distance:", "%.2f", distanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();

            if (colorSensor.alpha() > 800 && colorSensor.blue() > 300){
                dc = DetectedColor.White;
            }

            if (colorSensor.alpha() > 800 && colorSensor.blue() < 200){
                dc = DetectedColor.Yellow;
            }


            stop = timeout == 0 || (timeout > 0 && runtime.seconds() >= timeout);
        }

       // toggleLight(false);
        return dc;
    }

    public double getDistanceToClosestObject(){
        double dist = -1;
        double val = distanceSensor.getDistance(DistanceUnit.INCH);
        if(!Double.isNaN(val)){
            dist = val;
        }
        return dist;
    }
}
