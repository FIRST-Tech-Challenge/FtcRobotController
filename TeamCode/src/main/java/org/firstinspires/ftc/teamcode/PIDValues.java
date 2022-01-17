package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class PIDValues extends LinearOpMode {

    // Motors
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    double currentVelocity;

    double maxVelocity = 0;

    @Override
    public void runOpMode() {

        // Get the motors
        frontLeft = hardwareMap.get(DcMotorEx.class,"FrontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class,"FrontRight");
        backLeft = hardwareMap.get(DcMotorEx.class,"BackLeft");
        backRight = hardwareMap.get(DcMotorEx.class,"BackRight");

        waitForStart();

        while (opModeIsActive()) {

            frontLeft.setPower(1);

            currentVelocity = frontLeft.getVelocity();

            if (currentVelocity > maxVelocity) {

                maxVelocity = currentVelocity;

            }

            telemetry.addData("current velocity", currentVelocity);
            telemetry.addData("max velocity", maxVelocity);
            telemetry.update();

        }


    }


}
