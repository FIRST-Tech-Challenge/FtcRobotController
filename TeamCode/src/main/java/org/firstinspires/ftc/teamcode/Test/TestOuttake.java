package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class TestOuttake extends LinearOpMode {

    private DcMotor motor1;
    private Servo servo1;
    boolean ct = false;
    boolean prevInput = false;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean FlapOpenOrClosed = true;
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.a && prevInput == false) {
                ct = !ct;
                prevInput = true;
            }

            prevInput = gamepad1.a;

            if (ct == true) {
                FlapOpen();
            } else {
                FlapClosed();
            }
        }
    }
    public void FlapOpen() {
        motor1 = hardwareMap.dcMotor.get("motorOuttake");
        servo1 = hardwareMap.servo.get("servoOuttake");
        motor1.setPower(0.5);
        servo1.setPosition(0.5);
    }

    public void FlapClosed() {
        motor1 = hardwareMap.dcMotor.get("motorOuttake");
        servo1 = hardwareMap.servo.get("servoOuttake");
        motor1.setPower(0.5);
        servo1.setPosition(0.0);
    }

}
