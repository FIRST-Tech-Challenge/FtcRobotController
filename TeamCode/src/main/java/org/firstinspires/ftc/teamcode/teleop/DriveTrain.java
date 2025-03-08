package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Teleop DriveTrain", group="opmode")
public class DriveTrain extends OpMode {
    private final static double CRAWL_MODE = 0.35;
    private final static double NORMAL_MODE = 0.85;
    private DcMotorEx leftFront, leftBack, rightBack, rightFront;


    @Override
    public void init() {
        telemetry.addData("Status", "Initialization Begin");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        // reverse any motors using DcMotor.setDirection()
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialization End");
    }

    @Override
    public void loop() {
        double leftFrontPower, leftBackPower, rightFrontPower, rightBackPower;

        double y = gamepad1.left_stick_y * -1;
        double x = gamepad1.left_stick_x * 1.5;
        double pivot = gamepad1.right_stick_x;

        leftFrontPower = pivot + y + x;
        leftBackPower = pivot + y - x;
        rightFrontPower = -pivot + y - x;
        rightBackPower = -pivot + y + x;

        if (gamepad1.left_bumper) {
            leftFront.setPower(leftFrontPower * CRAWL_MODE);
            leftBack.setPower(leftBackPower * CRAWL_MODE);
            rightFront.setPower(rightFrontPower * CRAWL_MODE);
            rightBack.setPower(rightBackPower * CRAWL_MODE);
        } else {
            leftFront.setPower(leftFrontPower * NORMAL_MODE);
            leftBack.setPower(leftBackPower * NORMAL_MODE);
            rightFront.setPower(rightFrontPower * NORMAL_MODE);
            rightBack.setPower(rightBackPower * NORMAL_MODE);
        }
    }
}
