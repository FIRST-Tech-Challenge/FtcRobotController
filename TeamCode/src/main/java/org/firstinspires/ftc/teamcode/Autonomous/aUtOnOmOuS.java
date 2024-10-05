package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Systems.Motors;

@Autonomous(name="Main")

public class aUtOnOmOuS extends LinearOpMode {

    Motors motor;

    public void runOpMode() throws InterruptedException {

        //Initialize stuff
        motor.Initialize();

        waitForStart();
        //run once

        while (opModeIsActive())
        {
            motor.MoveMotor(0, 100);
            //Runs here repeatedly
        }
    }
}
