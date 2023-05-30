package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MareaMiscare extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        //Declare motors for drivetrain
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("Frontleft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("Backleft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("Frontright");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("Backright");

        //Declare lift
        DcMotor LiftP = hardwareMap.dcMotor.get("Lift");

        //Reverse the motors for drivetrain
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            double lift = -gamepad1.right_stick_y /2;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;



            motorFrontLeft.setPower(frontLeftPower/2);
            motorBackLeft.setPower(backLeftPower/2);
            motorFrontRight.setPower(frontRightPower/2);
            motorBackRight.setPower(backRightPower/2);
            LiftP.setPower(lift);
        }
    }
}