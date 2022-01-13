package org.firstinspires.ftc.teamcode.gen1_Qualifying;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(preselectTeleOp = "Beta_TeleOp")
public class Final_BlueDuckCarousel extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class,"frontRightMotor");
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class,"frontLeftMotor");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class,"backRightMotor");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class,"backLeftMotor");

        CRServo rightSpinner = hardwareMap.get(CRServo.class,"rightSpinner");
        CRServo leftSpinner = hardwareMap.get(CRServo.class,"leftSpinner");

        boolean delay = false;

        //Asks for delay
        while (!super.isStarted() && !super.isStopRequested()) {
            if (gamepad1.x) delay = true;
            else if (gamepad1.y) delay = false;
            telemetry.addData("Delay (switch with x or y)", delay);
            telemetry.update();
        }

        if (opModeIsActive()) {
            if (delay) sleep(3000);

            frontRightMotor.setPower(0.45);
            frontLeftMotor.setPower(-0.45);
            backRightMotor.setPower(0.5);
            backLeftMotor.setPower(0.5);

            sleep(400);

            frontRightMotor.setPower(0.25);
            frontLeftMotor.setPower(0.30);
            backRightMotor.setPower(0.25);
            backLeftMotor.setPower(-0.30);

            sleep(1800);

            frontRightMotor.setPower(0.0);
            frontLeftMotor.setPower(0.0);
            backRightMotor.setPower(0.0);
            backLeftMotor.setPower(0.0);

            sleep(1000);

            frontRightMotor.setPower(-0.2);
            frontLeftMotor.setPower(0.2);
            backRightMotor.setPower(-0.2);
            backLeftMotor.setPower(-0.25);
            rightSpinner.setPower(0.5);
            leftSpinner.setPower(-0.5);

            sleep(4000);

            frontRightMotor.setPower(0.4);
            frontLeftMotor.setPower(-0.4);
            backRightMotor.setPower(0.4);
            backLeftMotor.setPower(0.4);

            sleep (800);

            frontRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            backLeftMotor.setPower(0);


        }

    }


}
