package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

public class TwoPosServo extends LinearOpMode {

    private Servo servo = null;
    private double max = 0.01; // Maximum rotational position
    private double min = 0.99; // Minimum rotational position


    private String currentPos = "min";

    public TwoPosServo (Servo s, double mininput, double maxinput) {
        servo = s;
        max = maxinput;
        min = mininput;
    }

    public void minPos() {
        servo.setPosition(min);
    }
    public void maxPos() {
        servo.setPosition(max);
    }

    public void nextPos() {
        if(currentPos == "min") {
            currentPos = "max";
            maxPos();
        } else if(currentPos == "max") {
            currentPos = "min";
            minPos();
        }
    }

    public String getPos() {
        return currentPos;
    }

    public void runOpMode() {

    }
}
