package org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "NEW Basic Auto!")
public class BasicAuto extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        // Assign a variable to the front left motor
        DcMotor fleft = hardwareMap.get(DcMotor.class, "fleft");

        waitForStart(); // Waits for a person to press start on the control hub
        // then it runs the rest of the program

        fleft.setPower(0.3); // sets the power of one of the motors

        timerFunction(10000);

        // stopFunction();


    }


    public void stopFunction() {
        while (opModeIsActive()) {

        }
    }

    public void timerFunction(int duration) {
        long startTime = System.currentTimeMillis(); // Stores the time

        while (System.currentTimeMillis() < startTime + duration && opModeIsActive()) {

        }
    }


}