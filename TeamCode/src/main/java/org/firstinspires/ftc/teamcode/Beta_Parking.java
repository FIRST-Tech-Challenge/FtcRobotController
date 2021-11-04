package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(preselectTeleOp = "Beta_TeleOp")
public class Beta_Parking extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class,"frontRightMotor");
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class,"frontLeftMotor");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class,"backRightMotor");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class,"backLeftMotor");

        waitForStart();
        if (opModeIsActive()) {

            frontRightMotor.setPower(-0.4);
            frontLeftMotor.setPower(-0.55);
            backRightMotor.setPower(0.4);
            backLeftMotor.setPower(-0.55);

            sleep(500);

            frontRightMotor.setPower(0.0);
            frontLeftMotor.setPower(0.0);
            backRightMotor.setPower(0.0);
            backLeftMotor.setPower(0.0);

        }


    }


}
