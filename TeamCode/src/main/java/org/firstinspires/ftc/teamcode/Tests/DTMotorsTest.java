package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class DTMotorsTest extends LinearOpMode {
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class, "fl");
        leftBack = hardwareMap.get(DcMotorEx.class, "bl");
        rightFront = hardwareMap.get(DcMotorEx.class, "fr");
        leftFront = hardwareMap.get(DcMotorEx.class, "br");

        waitForStart();
        while(opModeIsActive()){
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double r = gamepad1.right_stick_x;
            double normalize = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(r), 1);
            double flPower = (y+x-r);
            double blPower = (y-x+r);
            double brPower = (y+x+r);
            double frPower = (y-x-r);
            leftFront.setPower((flPower/normalize));
            leftBack.setPower((blPower/normalize));
            rightBack.setPower((brPower/normalize));
            rightFront.setPower((frPower/normalize));
        }
    }
}
