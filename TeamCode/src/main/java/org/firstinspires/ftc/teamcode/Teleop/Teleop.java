package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Teleop")
@Disabled
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeft = hardwareMap.dcMotor.get("fl");
        DcMotor backLeft = hardwareMap.dcMotor.get("bl");
        DcMotor frontRight = hardwareMap.dcMotor.get("fr");
        DcMotor backRight = hardwareMap.dcMotor.get("br");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) { // clearer nomenclature for variables
            double ly = gamepad1.left_stick_y;
            double lx = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double ry = gamepad1.right_stick_y;

            double r = Math.hypot(lx, ly);
            double robotAngle = Math.atan2(ly, lx) - Math.PI / 4;
            double lrPower = r * Math.sin(robotAngle) + rx;
            double lfPower = r * Math.cos(robotAngle) + rx;
            double rrPower = r * Math.cos(robotAngle) - rx;
            double rfPower = r * Math.sin(robotAngle) - rx;
            frontLeft.setPower(lfPower);
            frontRight.setPower(rfPower);
            backLeft.setPower(lrPower);
            backRight.setPower(rrPower);
        }
    }
}
