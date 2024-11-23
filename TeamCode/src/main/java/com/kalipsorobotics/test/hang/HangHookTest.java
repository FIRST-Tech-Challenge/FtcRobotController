package com.kalipsorobotics.test.hang;

import com.kalipsorobotics.actions.hang.HangHooks;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class HangHookTest extends LinearOpMode {
    LinearOpMode linearOpMode;
    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, linearOpMode, telemetry);
        HangHooks hangHooks = new HangHooks(opModeUtilities);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("right hook", hangHooks.getHookPosition("r"));
            telemetry.addData("left hook", hangHooks.getHookPosition("l"));
            telemetry.update();
        }
    }
}
