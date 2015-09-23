package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by Jordan Burklund on 7/30/2015.
 * An example linear op mode where the pushbot 
 * will run its motors unless a touch sensor 
 * is pressed.
 */
public class PushBotDriveTouch extends LinearOpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;
    TouchSensor touchSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        // Get references to the motors from the hardware map
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");

        // Reverse the right motor
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Get a reference to the touch sensor
        touchSensor = hardwareMap.touchSensor.get("sensor_touch");

        // Wait for the start button to be pressed
        waitForStart();

        while(opModeIsActive()) {
            if(touchSensor.isPressed()) {
                //Stop the motors if the touch sensor is pressed
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            } else {
                //Keep driving if the touch sensor is not pressed
                leftMotor.setPower(0.5);
                rightMotor.setPower(0.5);
            }

            telemetry.addData("isPressed", String.valueOf(touchSensor.isPressed()));

            // Wait for a hardware cycle to allow other processes to run
            waitOneFullHardwareCycle();
        }

    }
}
