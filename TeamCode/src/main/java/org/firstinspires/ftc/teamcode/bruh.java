package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "basic teleop", group = "teamcode")
public class bruh extends OpMode {

    DcMotor backleft, backright;
    Servo extend, wrist;
    CRServo intake;


    @Override
    public void init() {
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        extend = hardwareMap.get(Servo.class, "extend");
        wrist = hardwareMap.get(Servo.class, "wrist");
        intake = hardwareMap.get(CRServo.class, "intake");
    }

    @Override
    public void loop() {
        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;

        backleft.setPower(Range.clip(y + x, -1, 1));
        backright.setPower(Range.clip(y - x, -1, 1));

        if (gamepad2.right_bumper) {

            intake.setPower(1);
            extend.setPosition(0.5);
            wrist.setPosition(0.25);

        }
        else{
            extend.setPosition(0);
            wrist.setPosition(0);
        }


    }
}