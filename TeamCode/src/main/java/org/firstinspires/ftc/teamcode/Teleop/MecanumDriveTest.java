package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="MecanumDriveTest")
public class MecanumDriveTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("fl");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("bl");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("fr");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("br");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double r = Math.hypot(x, y);
            double robotAngle = Math.atan2(y, x) - Math.PI / 4;
            double lrPower = r * Math.sin(robotAngle) + rx;
            double lfPower = r * Math.cos(robotAngle) + rx;
            double rrPower = r * Math.cos(robotAngle) - rx;
            double rfPower = r * Math.sin(robotAngle) - rx;

            motorFrontLeft.setPower(lfPower);
            motorBackLeft.setPower(lrPower);
            motorFrontRight.setPower(rfPower);
            motorBackRight.setPower(rrPower);
        }
    }
}