package com.kalipsorobotics.test.hang;

import com.kalipsorobotics.actions.hang.HangHooks;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class HangHookTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        HangHooks hangHooks = new HangHooks();
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("right hook", hangHooks.getHookPosition("r"));
            telemetry.addData("left hook", hangHooks.getHookPosition("l"));
        }
    }
}
