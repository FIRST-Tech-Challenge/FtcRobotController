package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.vision.TurretCamera;

@TeleOp(name = "HiveAimTest", group = "Linear OpMode")
public class HiveAimTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // The vision test needs only the hive webcam. The drivetrain and odometry
        // can stay disconnected while we test on the bench.
        TurretCamera camera = new TurretCamera(hardwareMap);
        try {
            camera.setAlliance(TurretCamera.Alliance.RED);
            String captureStatus = "Press X to capture a raw PNG";
            // Let the team calibrate during INIT or after START.
            while (!isStopRequested()) {
                camera.periodic();
                if (gamepad1.aWasPressed()) camera.setAlliance(TurretCamera.Alliance.RED);
                if (gamepad1.bWasPressed()) camera.setAlliance(TurretCamera.Alliance.BLUE);
                if (gamepad1.dpadUpWasPressed()) camera.adjustLighting(1, 0);
                if (gamepad1.dpadDownWasPressed()) camera.adjustLighting(-1, 0);
                if (gamepad1.dpadRightWasPressed()) camera.adjustLighting(0, 5);
                if (gamepad1.dpadLeftWasPressed()) camera.adjustLighting(0, -5);
                if (gamepad1.yWasPressed()) camera.resetLighting();
                if (gamepad1.rightBumperWasPressed()) camera.saveLighting();
                if (gamepad1.xWasPressed()) {
                    String filename = camera.requestRawFrame();
                    captureStatus = filename == null
                            ? "Camera is not streaming"
                            : "Requested " + filename;
                }
                telemetry.addLine("A red / B blue / X raw PNG");
                telemetry.addLine("D-pad up/down exposure / left/right gain");
                telemetry.addLine("Y automatic exposure / right bumper save");
                camera.writeLightingTelemetry(telemetry);
                camera.writeTelemetry(telemetry);
                telemetry.addData("Raw capture", captureStatus);
                telemetry.update();
                idle();
            }
        } finally {
            camera.shutdown();
        }
    }
}
