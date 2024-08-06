package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.concurrent.TimeUnit;

@TeleOp(group = "ZTest")
public class MotorTestSimple extends LinearOpMode{
    public void runOpMode() throws InterruptedException {
        DcMotor motor = hardwareMap.dcMotor.get("m");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                motor.setPower(-0.5);
            } else if (gamepad1.b) {
                motor.setPower(0.5);
            } else if (gamepad1.x) {
                motor.setPower(0.0);
            }
        }
    }
}