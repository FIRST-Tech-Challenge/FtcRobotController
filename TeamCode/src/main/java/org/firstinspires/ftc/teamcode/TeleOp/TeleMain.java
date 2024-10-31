package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.Motors;
import org.firstinspires.ftc.teamcode.Systems.Servos;

@TeleOp(name="Main")
public class TeleMain extends LinearOpMode {

    Motors motors;
    Servos servos;

    @Override
    public void runOpMode() throws InterruptedException {

        motors = new Motors(hardwareMap);
        servos = new Servos(hardwareMap);

        /*

        Run anything you put here once
        You can put things here that need to be initialized
        Usually variables would go here
        For example creating string = ">:)";
        */
        waitForStart();

        while (opModeIsActive())
        {
            /*

            Repeat anything here over and over and over and over and over and over...
            Put the actual part of the code here
            For example detecting when a gamepad moves a joystick to move a motor


             */
        }
    }
}
