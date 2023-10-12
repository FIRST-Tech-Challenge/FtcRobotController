package org.firstinspires.ftc.teamcode.OpMode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class DrivechainTester extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {

        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double leftStick = gamepad1.left_stick_y;
        double rightStick = gamepad1.right_stick_x;

        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()) {

            double y = -leftStick; // Remember, this is reversed!
            double x = rightStick * 1.1; // Counteract imperfect strafing
            double rx = rightStick;
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) * 0.65/ denominator;
            double backLeftPower = (y - x + rx) * 0.65/ denominator;
            double frontRightPower = (y - x - rx) * 0.65/ denominator;
            double backRightPower = (y + x - rx)* 0.65/ denominator;
            frontLeft.setPower(-frontLeftPower*0.6);
            backLeft.setPower(-backLeftPower*0.6);
            frontRight.setPower(-frontRightPower*0.6);
            backRight.setPower(-backRightPower*0.6);

            telemetry.addData("TEST VALUES", null);
            telemetry.addData("LEFT STICK Y - FWD BACK DRIVE (REMEMBER THAT THIS IS OPPOSITE OF DIRECTION OF STICK)", leftStick);
            telemetry.addData("RIGHT STICK X - STRAFING", rightStick);
            telemetry.addData("frontLeftPower:", frontLeftPower);
            telemetry.addData("frontRightPower:", frontRightPower);
            telemetry.addData("backLeftPower:", backLeftPower);
            telemetry.addData("backRightPower:", backRightPower);
            telemetry.update();

        }
    }
}
