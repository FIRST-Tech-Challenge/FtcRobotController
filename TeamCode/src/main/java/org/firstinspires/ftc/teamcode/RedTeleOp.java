package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class RedTeleOp extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {

        // initialize robot class
        robot = new Robot(hardwareMap, this, telemetry, true, true, false);
        robot.initForTeleOp();

        waitForStart();

        //TODO: add manual pivot
        robot.teleOpWhileLoop(gamepad1, gamepad2);
    }
}
