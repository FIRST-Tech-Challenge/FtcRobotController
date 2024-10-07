package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name= "hoco")
public class jarlsberg extends OpMode{

    DcMotorEx rightFront = null;
    DcMotorEx leftFront = null;
    DcMotorEx rightBack = null;
    DcMotorEx leftBack = null;

    double max = 0.0;

    @Override
    public void init()
    {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightRear");

        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
    }

    @Override
    public void loop() {

        float x = gamepad1.left_stick_x;
        float y = -gamepad1.left_stick_y;
        float rx = gamepad1.right_stick_x;

        double leftFrontPower = y+x+rx;
        double rightFrontPower = y-x-rx;
        double leftBackPower = y-x+rx;
        double rightBackPower = y+x-rx;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }


        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);

    }


}
