package org.firstinspires.ftc.team15091;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;

public abstract class AutonomousBase extends OpModeBase {
    protected RobotDriver robotDriver;
    protected PixelPosition pixelPos = PixelPosition.Left;
    protected VisionPortal visionPortal;
    protected AprilTagDetector aprilTagDetector;
    protected RBProcessor rbProcessor;
    protected long delay_start = 0;
    public boolean should_park = true;
    View relativeLayout;
    final protected void setupAndWait() {
        robot.init(hardwareMap);
        robotDriver = new RobotDriver(robot, this);
        aprilTagDetector = new AprilTagDetector();
        rbProcessor = new RBProcessor();
        aprilTagDetector.init();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagDetector.aprilTag)
                .addProcessor(rbProcessor)
                .build();

        telemetry.addData("Heading", "%.4f", () -> robot.getHeading());
        if (Math.abs(robot.getHeading()) > 20) {
            robot.beep(1);
        }
        telemetry.addLine("Pixel | ")
                .addData("pos", "%s", () -> pixelPos.toString());
        telemetry.addLine("Sensor | ")
                .addData("limit", () -> String.format("%s", robot.limitSwitch.getState()))
                .addData("distance", "%.1f cm", () -> robot.frontSensor.getDistance(DistanceUnit.CM));

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {

            if (gamepad1.dpad_up || gamepad2.dpad_up) {
                if (!dpad_pressed) {
                    delay_start += 100;
                }
            } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
                if (!dpad_pressed) {
                    if (delay_start > 0) {
                        delay_start -= 100;
                    }
                }
                if (gamepad1.x) {
                    should_park = false;
                }
            }

            if (gamepad1.a) {
                if (!a_pressed) {
                    a_pressed = true;
                }
            } else {
                a_pressed = false;
            }

            pixelPos = rbProcessor.position;
            telemetry.update();
            idle();
        }

        if (delay_start > 0d) {
            sleep(delay_start);
        }
    }
}
