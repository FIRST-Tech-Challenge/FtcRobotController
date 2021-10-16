package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class CarouselBeta extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor rightSpinner = hardwareMap.get(DcMotor.class,"rightSpinner");
        DcMotor leftSpinner = hardwareMap.get(DcMotor.class,"leftSpinner");

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
