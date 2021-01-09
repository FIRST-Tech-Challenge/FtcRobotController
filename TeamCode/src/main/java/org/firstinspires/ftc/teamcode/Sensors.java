package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import android.graphics.Color;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Sensors extends LinearOpMode {

    //init the sensors
    public Rev2mDistanceSensor distancefront = null;
    public Rev2mDistanceSensor distanceback = null;
    public Rev2mDistanceSensor distanceleft = null;
    public Rev2mDistanceSensor distanceright = null;

    public ColorSensor colorleft = null;
    public ColorSensor colorright = null;
    float hsvValuesLeft[] = {0F,0F,0F};
    float hsvValuesRight[] = {0F,0F,0F};

    public Sensors(Rev2mDistanceSensor f,
                   Rev2mDistanceSensor b,
                   Rev2mDistanceSensor l,
                   Rev2mDistanceSensor r,
                   ColorSensor cl,
                   ColorSensor cr) {
        distancefront = f;
        distanceback = b;
        distanceleft = l;
        distanceright = r;
        colorleft = cl;
        colorright = cr;
    }

    public Sensors(Rev2mDistanceSensor f,
                   Rev2mDistanceSensor b,
                   Rev2mDistanceSensor l,
                   Rev2mDistanceSensor r) {
        distancefront = f;
        distanceback = b;
        distanceleft = l;
        distanceright = r;
    }

    public Sensors(ColorSensor cl,
                   ColorSensor cr) {
        colorleft = cl;
        colorright = cr;
    }

    public Sensors(Rev2mDistanceSensor f) {
        distancefront = f;
    }

    public double getDistanceFront() {
        return distancefront.getDistance(DistanceUnit.INCH);
    }
    public double getDistanceBack() {
        return distanceback.getDistance(DistanceUnit.INCH);
    }
    public double getDistanceLeft() {
        return distanceleft.getDistance(DistanceUnit.INCH);
    }
    public double getDistanceRight() {
        return distanceright.getDistance(DistanceUnit.INCH);
    }
    public float[] getHSVLeft() {
        Color.RGBToHSV(colorleft.red()*8, colorleft.green()*8, colorleft.blue()*8, hsvValuesLeft);
        return hsvValuesLeft;
    }
    public float[] getHSVRight() {
        Color.RGBToHSV(colorright.red()*8, colorright.green()*8, colorright.blue()*8, hsvValuesRight);
        return hsvValuesRight;
    }
    public double[] getRGBLeft() {
        double[] a = {colorleft.red(), colorleft.green(), colorleft.blue()};
        return a;
    }
    public double[] getRGBRight() {
        double[] a = {colorright.red(), colorright.green(), colorright.blue()};
        return a;
    }

    public void runOpMode() {

    }

}
