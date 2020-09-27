package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MecanumTest")
public class MecanumTest extends OpMode {
    private DcMotor driveLeftFront, driveLeftBack, driveRightFront, driveRightBack;
    private double speedModifier = 1;

    @Override
    public void init() {
        driveLeftFront = hardwareMap.dcMotor.get("dlf");
        driveLeftBack = hardwareMap.dcMotor.get("dlb");
        driveRightFront = hardwareMap.dcMotor.get("drf");
        driveRightBack = hardwareMap.dcMotor.get("drb");
    }

    @Override
    public void loop() {


        double r = Math.hypot(-gamepad1.left_stick_y, gamepad1.left_stick_x);
        double robotAngle = Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y) + Math.PI / 4;
        double rightX = gamepad1.right_stick_x;

        final double a = r * Math.sin(robotAngle) + rightX;
        final double b = r * Math.cos(robotAngle) - rightX;
        final double c = r * Math.cos(robotAngle) + rightX;
        final double d = r * Math.sin(robotAngle) - rightX;

        driveLeftFront.setPower(-a);
        driveRightFront.setPower(b);
        driveRightBack.setPower(d);
        driveLeftBack.setPower(-c);

        telemetry.addData("Left Front", -a);
        telemetry.addData("Right Front", b);
        telemetry.addData("Left Back", -c);
        telemetry.addData("Right Back", d);

        // test change
    }
}