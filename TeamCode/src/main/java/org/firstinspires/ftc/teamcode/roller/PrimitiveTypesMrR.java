package org.firstinspires.ftc.teamcode.roller;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp //Assigns OpMode to the Teleop dropdown on DriverStation
public class PrimitiveTypesMrR extends OpMode {
    @Override
    public void init () {
        //create variable of type integer to store team number (17348)
        int teamNumber = 17348;

        //create a variable of type double to store motor speed (range = 0.0 to 1.0)
        double motorSpeed = 0.5;

        //create a boolean type variable for the touch sensor, assign value of true at initialization
        boolean touchSensorPressed = true;

        //write data stored in each variable to the Driver Station
        telemetry.addData("Team Number", teamNumber);
        telemetry.addData("Motor Speed", motorSpeed);
        telemetry.addData("Touch Sensor", touchSensorPressed);

    }
    @Override
    public void loop () {
        //empty loop; method always needed, even if empty

    }
}
