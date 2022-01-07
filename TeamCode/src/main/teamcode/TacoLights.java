package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//@TeleOp(name = "TacoLights")

public class TacoLights extends Taco_FF_Super_Class {

    @Override
    public void runOpMode() {
        initialization(false);
        Blinkyboi.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(76));
        waitForStart();
        if (opModeIsActive()) {
            start_time = getRuntime();
            Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            times_run++;
            while (opModeIsActive()) {
                drive();
                lift();
                intake();
                duck();
                time();
                color();
                telemetry.addData("Blue", blue);
                telemetry.addData("How many times we have run teleop", times_run);
                telemetry.update();
            }
        }
    }
}
