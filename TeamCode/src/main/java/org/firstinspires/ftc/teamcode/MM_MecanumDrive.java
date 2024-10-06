package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

    @TeleOp
   //@Disabled
    public class MM_MecanumDrive extends LinearOpMode {
        @Override
        public void runOpMode() throws InterruptedException {
            // Declare our motors\
            // Make sure your ID's match your configuration
            DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
            DcMotor rearLeft = hardwareMap.dcMotor.get("rearLeft");
            DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
            DcMotor rearRight = hardwareMap.dcMotor.get("rearRight");

            // Reverse the right side motors
            // Reverse left motors if you are using NeveRests
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            rearRight.setDirection(DcMotorSimple.Direction.REVERSE);
            //frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

            waitForStart();

            //if (isStopRequested()) return;

            while (opModeIsActive()) {
                double y = gamepad1.left_stick_y; // Remember, this is reversed!
                double x =  -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                double rx = -gamepad1.right_stick_x;

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio, but only when
                // at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

                double frontLeftPower = ( y - x +rx) / denominator;
                double rearLeftPower = ( y + x +rx) / denominator;
                double frontRightPower = ( y -x -rx) / denominator;
                double rearRightPower = ( y + x-rx) / denominator;
                //old code
//                double frontLeftPower = ( y + x +rx) / denominator;
//                double rearLeftPower = ( y - x +rx) / denominator;
//                double frontRightPower = ( y -x -rx) / denominator;
//                double rearRightPower = ( y + x-rx) / denominator;
                frontLeft.setPower(frontLeftPower);
                rearLeft.setPower(rearLeftPower);
                frontRight.setPower(frontRightPower);
                rearRight.setPower(rearRightPower);

            }
        }
    }
