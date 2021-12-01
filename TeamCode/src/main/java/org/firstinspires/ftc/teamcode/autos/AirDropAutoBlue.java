package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AirDropAutoBlue extends LinearOpMode {
    DcMotor frontLeft, frontRight, backLeft, backRight;
    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Comment these out to reverse motors if necessary
//        frontLeft.setDirection(DcMotor.Direction.REVERSE);
//        frontRight.setDirection(DcMotor.Direction.REVERSE);
//        backLeft.setDirection(DcMotor.Direction.REVERSE);
//        backRight.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 2) {
            frontLeft.setPower(.3);
            frontRight.setPower(.3);
            backLeft.setPower(.3);
            backRight.setPower(.3);
        }
        while (opModeIsActive() && runtime.seconds() < 4) {
            frontLeft.setPower(.3);
            frontRight.setPower(-.3);
            backLeft.setPower(.3);
            backRight.setPower(-.3);
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
