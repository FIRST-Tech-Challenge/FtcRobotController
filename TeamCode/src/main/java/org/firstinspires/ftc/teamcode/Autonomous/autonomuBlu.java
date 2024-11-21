package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Systems.Motors;

@Autonomous(name="Main")


public class autonomuBlu extends LinearOpMode {

    Motors motor;

    public void runOpMode() throws InterruptedException {
        motor = new Motors(hardwareMap);
[]
        waitForStart();
        //run once

        while (opModeIsActive())
        {
            motor.MoveMotor(0, 100);
            motor.MoveMotor(1, 100);
            motor.MoveMotor(2, 100);
            motor.MoveMotor(3, 100);
            sleep(350);
            motor.MoveMotor(0, 0);
            motor.MoveMotor(1, 0);
            motor.MoveMotor(2, 0);
            motor.MoveMotor(3, 0);

        }
    }
}
