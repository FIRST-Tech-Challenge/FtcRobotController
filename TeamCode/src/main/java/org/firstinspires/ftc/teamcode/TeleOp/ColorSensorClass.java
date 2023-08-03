package org.firstinspires.ftc.teamcode.TeleOp;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColorSensorClass
{
    ColorSensor colorSensor;
    private boolean bLedOn = true;
    private float hsvValues[] = {0F,0F,0F};
    private final float values[] = hsvValues;
    private boolean bPrevState = false;
    private boolean bCurrState = false;



    public ColorSensorClass(HardwareMap hardwareMap)
    {
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        colorSensor.enableLed(bLedOn);
    }
}
