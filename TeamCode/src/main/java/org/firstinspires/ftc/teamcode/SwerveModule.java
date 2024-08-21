package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class SwerveModule extends LinearOpMode {

    DcMotor motor1 = null;
    Servo servo1 = null;


    @Override
    public void runOpMode() throws InterruptedException {

        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        servo1 = hardwareMap.get(Servo.class, "servo1");

        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();
        while (opModeIsActive()) {

            motor1.setPower(-gamepad1.left_stick_y);
            servo1.setPosition((gamepad1.right_stick_x + 1) / 2);

        }
    }
}
