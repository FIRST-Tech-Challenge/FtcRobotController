package org.firstinspires.ftc.teamcode.Alan;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class MyTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        waitForStart();
        boolean prev = false;
        while (opModeIsActive()) {
            if (gamepad1.a && !prev) {
                timer.reset();

            }
            prev = gamepad1.a;

            telemetry.addData("time", timer);





            telemetry.update();
        }
    }
}
