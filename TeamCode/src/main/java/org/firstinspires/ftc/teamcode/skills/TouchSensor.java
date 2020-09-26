package org.firstinspires.ftc.teamcode.skills;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by sjeltuhin on 9/26/17.
 */

public class TouchSensor {
    DigitalChannel digitalTouch;


    public void init(HardwareMap hardwareMap){
        // Get a reference to our sensor object.
        digitalTouch = hardwareMap.get(DigitalChannel.class, "touch_sensor");

    }

    public boolean isPressed(){
        return digitalTouch.getState() == false;
    }
}
