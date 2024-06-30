package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class drive extends LinearOpMode {
    private DcMotor leftMotor; // location 2
    private DcMotor rightMotor; // location 1
    private DcMotor middleMotor; // location 0
    @Override
    public void runOpMode()  {

        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        middleMotor = hardwareMap.get(DcMotor.class, "middleMotor");

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while(opModeIsActive()) {
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double leftPower = (y + x + rx) / denominator;
            double rightPower = (y - x - rx) / denominator;
            double middlePower = (x) / denominator;


            if (x > 0.2) {
                middleMotor.setPower(middlePower);
            } else {
                leftMotor.setPower(leftPower);
                rightMotor.setPower(rightPower);
            }



        }

    }
}
