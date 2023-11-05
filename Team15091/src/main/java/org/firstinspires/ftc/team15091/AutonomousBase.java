package org.firstinspires.ftc.team15091;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public abstract class AutonomousBase extends OpModeBase {
    protected RobotDriver robotDriver;
    protected YellowDetector pixelDetector;
    protected PixelPosition pixelPos = PixelPosition.Left;
    protected long delay_start = 0;
    public boolean should_park = true;
    View relativeLayout;
    final protected void setupAndWait() {
        robot.init(hardwareMap);
        robotDriver = new RobotDriver(robot, this);
        pixelDetector = new YellowDetector(hardwareMap);

        telemetry.addData("Heading", "%.4f", () -> robot.getHeading());
        if (Math.abs(robot.getHeading()) > 20) {
            robot.beep(1);
        }
        telemetry.addLine("Pixel | ")
                .addData("pos", "%s", () -> pixelPos.toString())
                .addData("debug", "%s", () -> pixelDetector.debug());
        telemetry.addLine("Front Distance ").addData("", "%.4f", () -> robot.frontSensor.getDistance(DistanceUnit.CM));

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

            gamepadUpdate();
            pixelPos = pixelDetector.objectDetected();
            telemetry.update();
            idle();
        }

        if (delay_start > 0d) {
            sleep(delay_start);
        }
    }
}
