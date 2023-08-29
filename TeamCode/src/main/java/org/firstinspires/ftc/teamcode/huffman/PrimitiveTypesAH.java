package org.firstinspires.ftc.teamcode.huffman;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp() //Assigns OpMode to the TeleOp dropdown on DriverStation
public class PrimitiveTypesAH extends OpMode {
    @Override
    public void init() {
        //create variable of type integer to store team number
        int teamNumber = 16072;

        //create variable of type double to store motorspeed
        double motorSpeed = 0.5;

        //create variable of type boolean for the touch sensor to assign value of true at initialization
        boolean touchSensorPressed = true;

        //writes data stored in each variable to the Driver Station
        telemetry.addData("Team Number", teamNumber);
        telemetry.addData("Motor Speed", motorSpeed);
        telemetry.addData("Touch Sensor", touchSensorPressed);
    }

    @Override
    public void loop() {
//empty loop, method always needed, even if empty
    }
}