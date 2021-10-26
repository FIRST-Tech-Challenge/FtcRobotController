package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Test_Carousel extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        CRServo rightSpinner = hardwareMap.get(CRServo.class,"rightSpinner");
        CRServo leftSpinner = hardwareMap.get(CRServo.class,"leftSpinner");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                double speed = 0.5; //figure out the speed for the Ducks
                rightSpinner.setPower(speed);
                leftSpinner.setPower(-speed);
            } else {
                rightSpinner.setPower(0.0);
                leftSpinner.setPower(0.0);
            }


        }


    }


}
