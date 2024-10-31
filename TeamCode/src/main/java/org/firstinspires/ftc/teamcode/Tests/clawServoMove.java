package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Systems.Servos;

@Autonomous(name = "servo test")
public class clawServoMove extends LinearOpMode {

    Servos servos;
    int count = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        servos = new Servos(hardwareMap);

        waitForStart();

        while(opModeIsActive())
        {
            servos.moveServo(0,180);
            sleep(500);
            servos.moveServo(0,0);
            sleep(500);
        }
    }
}