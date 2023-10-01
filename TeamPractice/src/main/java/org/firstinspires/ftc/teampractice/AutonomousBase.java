package org.firstinspires.ftc.teampractice;

import org.firstinspires.ftc.teampractice.examples.DistanceDetector;
import org.firstinspires.ftc.teampractice.examples.TapeDetector;
import org.firstinspires.ftc.teampractice.examples.TouchDetector;

public abstract class AutonomousBase extends OpModeBase {
    protected RobotDriver robotDriver;
    protected long delay_start = 0;
    protected TouchDetector frontDetector;
    protected DistanceDetector rearDetector, leftDetector, rightDetector;
    protected TapeDetector tapeDetector;
    protected int highPolePos = 1925, mediumPolePos = 1600, lowPolePos = 805, junctionPos = 200, currentTarget = 0, cone5Pos = 270;

    final protected void setupAndWait() {
        robot.init(hardwareMap, true);
        robotDriver = new RobotDriver(robot, this);
        frontDetector = new TouchDetector(robot.frontSensor);
        rearDetector = new DistanceDetector(robot.rearSensor, 74d, true);
        leftDetector = new DistanceDetector(robot.leftSensor, 20d);
        rightDetector = new DistanceDetector(robot.rightSensor, 22d);

        tapeDetector = new TapeDetector(robot.colorSensor);

        robot.setGrabber(0);
        sleep(100L);
        robot.setGrabber(1);

        robot.setFrontServo(false);

        telemetry.addData("Heading", "%.4f", () -> robot.getHeading());
        telemetry.addLine()
                .addData("Front", "%b", () -> frontDetector.isPressed())
                .addData("Rear", "%.01f cm", () -> rearDetector.getCurrentDistance());
        telemetry.addLine()
                .addData("Left", "%.01f cm", () -> leftDetector.getCurrentDistance())
                .addData("Right", "%.01f cm", () -> rightDetector.getCurrentDistance());
        telemetry.addData("Delay", "%d", () -> delay_start);
        telemetry.addLine()
                .addData("Red", "%d", () -> robot.colorSensor.red())
                .addData("Blue", "%d", () -> robot.colorSensor.blue());

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
            }

            gamepadUpdate();

            telemetry.update();
            idle();
        }

        if (delay_start > 0d) {
            sleep(delay_start);
        }
    }
}
