package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Systems.Servos;

@Autonomous(name = "servo test")
public class clawServoMove extends LinearOpMode {

    Servos servos;


    @Override
    public void runOpMode() throws InterruptedException {

        servos = new Servos(hardwareMap);


        waitForStart();

        servos.moveServo(0,0);
        sleep(3000);

        while(opModeIsActive())
        {


            servos.moveServo(0,90);
            sleep(500);
            servos.moveServo(0,180);
            sleep(500);
            servos.moveServo(0,270);
            sleep(500);
            servos.moveServo(0,0);
            sleep(500);
        }
    }
}