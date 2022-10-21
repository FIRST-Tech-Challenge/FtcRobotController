package org.firstinspires.ftc.teamcode.Functions;



import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import android.graphics.ColorMatrixColorFilter;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;


public class ColorSensorClass {

    ColorSensor REVColor;
    public ColorSensorClass(ColorSensor _REVSensor){
        REVColor = _REVSensor;
    }
    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;


    //senzorul verfica daca culoare pe care o vede se afla in intervalul RGB pentru galben +/-
    public boolean checkForYellow(){
        Color.RGBToHSV((int) (REVColor.red() * SCALE_FACTOR),
                (int) (REVColor.green() * SCALE_FACTOR),
                (int) (REVColor.blue() * SCALE_FACTOR),
                hsvValues);
        if(REVColor.red() <= 255 && REVColor.red() >= 128 && REVColor.green() <= 255 && REVColor.green() >= 128 && REVColor.blue() < 50)
            return true;
        else return false;
    }

    //senzorul verfica daca culoare pe care o vede se afla in intervalul RGB pentru alb +/-
    public boolean checkForWhite(){
        //if(REVColor.red() <= 255 && REVColor.red() >= 230 && REVColor.green() <=
                //255 && REVColor.green() >= 230 && REVColor.blue() <= 255 && REVColor.blue() >= 230){
        if(REVColor.alpha()>=20)
            return true;
        else return false;
        }

    }

