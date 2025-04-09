package org.firstinspires.ftc.team00000.v2.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team00000.v2.vision.ColorVisionSubsystem;
import org.opencv.core.Point;

import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Test: Color Vision Angle", group = "Test")
public class TestColorAngleAuto extends LinearOpMode {
    private ColorVisionSubsystem vision;
    private Servo wristServo;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize camera subsystem
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        vision = new ColorVisionSubsystem(webcam);

        // Initialize servo
        wristServo = hardwareMap.get(Servo.class, "wrist_drive");

        telemetry.addLine("Initialized, waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            vision.update();

            if (vision.hasTarget()) {
                double angle = vision.getAngle();
                Point center = vision.getCenter();

                // Clamp and map angle to servo range
                double clampedAngle = Math.max(-45, Math.min(45, angle));
                double wristPos = 0.5 + (clampedAngle / 45.0) * 0.3; // maps to range [0.2, 0.8]

                wristServo.setPosition(wristPos);

                telemetry.addLine("Target Detected");
                telemetry.addData("Angle", angle);
                telemetry.addData("Clamped", clampedAngle);
                telemetry.addData("Center", center.toString());
                telemetry.addData("Wrist Position", wristPos);
            } else {
                telemetry.addLine("No Target");
            }

            telemetry.update();
            sleep(100); // optional: slow down loop for clarity
        }

        vision.stop(); // gracefully shut down vision
    }
}
