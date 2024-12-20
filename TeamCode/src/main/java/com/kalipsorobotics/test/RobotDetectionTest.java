package com.kalipsorobotics.test;

import com.kalipsorobotics.tensorflow.RobotDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class RobotDetectionTest extends LinearOpMode {
    RobotDetector robotDetector = new RobotDetector(telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {
            if (robotDetector.isRobotDetected()) {
                telemetry.addLine("robot Detected");
            } else {
                telemetry.addLine("no robot detected");
            }
        }
    }
}
