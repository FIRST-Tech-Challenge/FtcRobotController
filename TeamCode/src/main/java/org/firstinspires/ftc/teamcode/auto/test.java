package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class test extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;


    public void runOpMode() throws InterruptedException {

        frontLeft = hardwareMap.get(DcMotor.class, "FLDrive");
        frontRight = hardwareMap.get(DcMotor.class, "FRDrive");
        backLeft = hardwareMap.get(DcMotor.class, "BLDrive");
        backRight = hardwareMap.get(DcMotor.class, "BRDrive");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        frontLeft.setPower(.5);
       // frontRight.setPower(.5);
       // backLeft.setPower(.5);
        backRight.setPower(.5);

        sleep(500);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }
}
