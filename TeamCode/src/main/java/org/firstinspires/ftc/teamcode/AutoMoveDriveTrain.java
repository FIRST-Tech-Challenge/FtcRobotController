package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class AutoMoveDriveTrain extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class,"frontRightMotor");
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class,"frontLeftMotor");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class,"backRightMotor");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class,"backLeftMotor");

        waitForStart();
        while (opModeIsActive()) {

            frontRightMotor.setPower(0.5);
            frontLeftMotor.setPower(-0.5);
            backRightMotor.setPower(0.5);
            backLeftMotor.setPower(-0.5);

            sleep(10);

            frontRightMotor.setPower(0.0);
            frontLeftMotor.setPower(0.0);
            backRightMotor.setPower(0.0);
            backLeftMotor.setPower(0.0);
        }

    }


}
