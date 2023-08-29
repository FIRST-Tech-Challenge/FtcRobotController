package org.firstinspires.ftc.teamcode.roller;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp //Assigns OpMode to the Teleop dropdown on DriverStation
public class PrimitiveTypesMrR extends OpMode {
    @Override
    public void init () {
        //creates a variable that stores team number
        int teamNumber = 17348;

        //creates a variable of type double to store motor speed (range = 0.0 to 1.0)
        double motorSpeed = 0.5;

        //creates a boolean type variable for the touch sensor
        boolean touchSensorPressed = true;

        //writes data stored in each variable to the Driver Station
        telemetry.addData("Team Number", teamNumber);
        telemetry.addData("Motor Speed", motorSpeed);
        telemetry.addData("Touch Sensor", touchSensorPressed);

    }
    @Override
    public void loop () {
        //empty loop; method always needed, even if empty

    }
}
