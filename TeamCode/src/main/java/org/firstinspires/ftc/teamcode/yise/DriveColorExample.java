package org.firstinspires.ftc.teamcode.yise;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.yise.Parameters;

public class DriveColorExample {
    public ColorSensor backSensor;
    public ColorSensor frontSensor;
    public enum Colors {
        WHITE,
        GREEN,
        YELLOW,
        PURPLE,
        NONE
    }

    public enum Pixel {
        BACK,
        FRONT
    }

    public DriveColorExample(HardwareMap hardwareMap) {
        backSensor = hardwareMap.get(ColorSensor.class, "backsensor");
        frontSensor = hardwareMap.get(ColorSensor.class, "frontsensor");
    }

    public float[] getRedColor() {
        float[] colors = new float[2];
        colors[1] = backSensor.red();
        return (colors);
    }
    public float[] getBlueColor() {
        float[] colors = new float[2];
        colors[0] = frontSensor.blue();
        colors[1] = backSensor.blue();
        return (colors);
    }
    public float[] getGreenColor() {
        float[] colors = new float[2];
        colors[0] = frontSensor.green();
        colors[1] = backSensor.green();
        return (colors);
    }


    public Colors getBackPixelColor() {
        //(R + G) / B
        float colorRatio = (getRedColor()[1] + getGreenColor()[1]) / getBlueColor()[1];

        //>4 is yellow
        if (getGreenColor()[1] > 500) {
            if (colorRatio >= 3.5) {
                return Colors.YELLOW;
            }
            //>2.5 is green
            else if (colorRatio >= 2) {
                return Colors.GREEN;
            }
            //>1.3 is white
            else if (colorRatio >= 1.3) {
                return Colors.WHITE;
            }
            //>0 is purple
            else if (colorRatio > 0) {
                return Colors.PURPLE;
            }
        }
        return Colors.NONE;
    }

    public Colors getFrontPixelColor() {
        //(R + G) / B
        float colorRatio = (getRedColor()[0] + getGreenColor()[0]) / getBlueColor()[0];

        //>2 is yellow
        if (getGreenColor()[0] > 500) {
            if (colorRatio >= 2) {
                return Colors.YELLOW;
            }
            //>1 is green
            else if (colorRatio >= 1) {
                return Colors.GREEN;
            }
            //>0.6 is white
            else if (colorRatio >= 0.6) {
                return Colors.WHITE;
            }
            //>0 is purple
            else if (colorRatio > 0) {
                return Colors.PURPLE;
            }
        }
        return Colors.NONE;
    }

}


