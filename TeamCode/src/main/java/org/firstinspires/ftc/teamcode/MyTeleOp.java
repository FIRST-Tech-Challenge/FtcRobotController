package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class MyTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        ElapsedTime elapsedTime = new ElapsedTime();
        double startTime = elapsedTime.milliseconds();
        double elapsedSinceStartTime;

        while (opModeIsActive()) {

            if(gamepad1.a == true) {
                startTime = elapsedTime.milliseconds();

            }

            elapsedSinceStartTime = elapsedTime.milliseconds() - startTime;
            telemetry.addLine("Program has run for " + (Math.round(elapsedSinceStartTime/1000)) + " seconds.");
            telemetry.update();


        }


    }
}
