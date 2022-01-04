package org.firstinspires.ftc.team6220_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Park", group = "Autonomous")
public class Park extends MasterOpMode{

    @Override
    public void runOpMode() {
        Initialize();

        servoGrabber.setPosition(0.0);
        pauseMillis(500);
        servoArm.setPosition(0.81);
        waitForStart();
        motorArm.setTargetPosition(550);
        motorArm.setPower(0.9);
        pauseMillis(500);
        motorBackLeft.setPower(0.8);
        motorBackRight.setPower(0.8);
        motorFrontLeft.setPower(0.8);
        motorFrontRight.setPower(0.8);
        pauseMillis(1200);
        motorBackLeft.setPower(-0.1);
        motorBackRight.setPower(-0.1);
        motorFrontLeft.setPower(-0.1);
        motorFrontRight.setPower(-0.1);
        pauseMillis(100);
        motorArm.setTargetPosition(-10);
        motorArm.setPower(0.9);
        pauseMillis(500);
    }
}