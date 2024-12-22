package com.kalipsorobotics.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.get(Servo.class, "intakeServoTestRobot");
        double pos = 0;

        waitForStart();

        while (opModeIsActive()) {
            servo.setPosition(pos);
            telemetry.addLine(String.valueOf(pos));
            telemetry.update();
            sleep(1000);
            pos += 0.05;
        }
    }
}
