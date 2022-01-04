package org.firstinspires.ftc.team6220_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedParkBlockPlace", group = "Autonomous")
public class RedParkBlockPLace extends MasterAutonomous {

    @Override
    public void runOpMode() {
        Initialize();

        servoGrabber.setPosition(0.0);
        pauseMillis(500);
        servoArm.setPosition(0.81);
        waitForStart();
        servoArm.setPosition(0.15);
        motorArm.setTargetPosition(900);
        motorArm.setPower(0.9);
        pauseMillis(500);
        motorBackLeft.setPower(0.6);
        motorBackRight.setPower(0.6);
        motorFrontLeft.setPower(0.6);
        motorFrontRight.setPower(0.6);
        pauseMillis(650);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        pauseMillis(100);
        motorBackLeft.setPower(0.1);
        motorBackRight.setPower(0.1);
        motorFrontLeft.setPower(0.1);
        motorFrontRight.setPower(0.1);
        pauseMillis(900);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        pauseMillis(100);
        servoGrabber.setPosition(0.7);
        pauseMillis(700);
        motorBackLeft.setPower(-0.3);
        motorBackRight.setPower(-0.3);
        motorFrontLeft.setPower(-0.3);
        motorFrontRight.setPower(-0.3);
        pauseMillis(500);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        pauseMillis(100);
        motorBackLeft.setPower(0.3);
        motorBackRight.setPower(-0.3);
        motorFrontLeft.setPower(0.3);
        motorFrontRight.setPower(-0.3);
        pauseMillis(1350);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        pauseMillis(100);
        motorBackLeft.setPower(0.8);
        motorBackRight.setPower(0.8);
        motorFrontLeft.setPower(0.8);
        motorFrontRight.setPower(0.8);
        pauseMillis(1500);
        motorBackLeft.setPower(-0.1);
        motorBackRight.setPower(-0.1);
        motorFrontLeft.setPower(-0.1);
        motorFrontRight.setPower(-0.1);
        pauseMillis(100);
        servoGrabber.setPosition(0.0);
        pauseMillis(100);
        servoArm.setPosition(0.81);
        motorArm.setTargetPosition(-10);
        motorArm.setPower(0.9);
        pauseMillis(700);
    }
}