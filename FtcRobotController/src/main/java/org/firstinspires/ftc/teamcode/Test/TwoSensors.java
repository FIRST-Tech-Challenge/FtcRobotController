package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp

public class TwoSensors extends LinearOpMode {

    DcMotor motorZ;

    TouchSensor digitalTouch;
    TouchSensor digitalTouchTwo;

    @Override
    public void runOpMode() {

        digitalTouch = hardwareMap.get(TouchSensor.class, "s_d");
        digitalTouchTwo = hardwareMap.get(TouchSensor.class, "s_z");

        waitForStart();

        while (opModeIsActive()) {
            motorZ = hardwareMap.get(DcMotor.class, "motorZ");

            telemetry.addData("Digital Touch sensor one pressed", digitalTouchTwo.isPressed());
            telemetry.addData("Digital Touch sensor two pressed", digitalTouch.isPressed());


            if (digitalTouch.isPressed() && digitalTouchTwo.isPressed()) {
                motorZ.setPower(0.3);

            } else {
                telemetry.addData("Digital Touch", "Is NOT Pressed");
                motorZ.setPower(0.0);
            }

            telemetry.update();

        }
    }
}
