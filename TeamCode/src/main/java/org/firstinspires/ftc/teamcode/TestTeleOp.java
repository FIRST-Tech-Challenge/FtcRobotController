package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(hardwareMap, this, telemetry, true, true, false);
        robot.initForTeleOp();

        boolean motorsStopped = false;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {

                robot.planeLauncherServo.setPosition(0.5);
                motorsStopped = false;

            } else if (gamepad1.b) {

                robot.trayAngle.setPosition(0.5);
                motorsStopped = false;

            } else if (gamepad1.x) {

                robot.stackAttachment.setPosition(0.5);
                motorsStopped = false;

            } else if (gamepad1.y) {

                robot.spikeServo.setPosition(0.5);
                motorsStopped = false;

            }
        }
    }
}
