package org.firstinspires.ftc.blackswan;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Auto")
public class giftAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");
        Servo closeClaw = hardwareMap.servo.get("closeClaw");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        closeClaw.setPosition(.22);
        waitForStart();

        frontLeft.setPower(-.5);
        backLeft.setPower(.5);
        frontRight.setPower(.5);
        backRight.setPower(-.5);
        sleep(200);

        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        sleep(300);

        frontLeft.setPower(.5);
        backLeft.setPower(.5);
        frontRight.setPower(.5);
        backRight.setPower(.5);
        sleep(1400);

        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        frontLeft.setPower(.5);
        backLeft.setPower(-.5);
        frontRight.setPower(-.5);
        backRight.setPower(.5);
        sleep(200);

        closeClaw.setPosition(moo.closeClawVal);
        sleep(300);

        frontLeft.setPower(-.5);
        backLeft.setPower(-.5);
        frontRight.setPower(-.5);
        backRight.setPower(-.5);
        sleep(270);

        closeClaw.setPosition(.22);
    }
}
