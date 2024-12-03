package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware.MID_SERVO;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "ServoTuner")
public class ServoTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Get a list of all the servo names in the robot configuration
        List<String> servoNamesList = new ArrayList<String>();
        for (Servo servo1 : hardwareMap.servo) {
            //Devices can have multiple names, so we concatenate them with a colon
            String concatenatedNames = String.join(":", hardwareMap.getNamesOf(servo1));
            servoNamesList.add(concatenatedNames);
        }
        String[] servoNames = servoNamesList.toArray(new String[0]);

        int selectedIndex = 0;
        Servo servo = null;
        boolean dpadUpPressed = false;
        boolean dpadDownPressed = false;
        boolean dpadLeftPressed = false;
        boolean dpadRightPressed = false;
        double servoPosition = MID_SERVO;
        double increment = 0.1; // Start with tenths

        telemetry.addData("Instructions", "Use D-pad up/down to select a servo.");
        telemetry.update();

        // Allow the user to select a servo before start is pressed
        while (!isStarted()) {
            if (gamepad1.dpad_up && !dpadUpPressed) {
                selectedIndex = (selectedIndex - 1 + servoNames.length) % servoNames.length;
                dpadUpPressed = true;
            } else if (!gamepad1.dpad_up) {
                dpadUpPressed = false;
            }

            if (gamepad1.dpad_down && !dpadDownPressed) {
                selectedIndex = (selectedIndex + 1) % servoNames.length;
                dpadDownPressed = true;
            } else if (!gamepad1.dpad_down) {
                dpadDownPressed = false;
            }

            telemetry.addData("Instructions", "Use D-pad up/down to select a servo.");
            telemetry.addData("Selected Servo", servoNames[selectedIndex]);
            telemetry.update();
        }

        servo = hardwareMap.tryGet(Servo.class, servoNames[selectedIndex]);

        waitForStart();

        // Allow the user to control the servo during the main loop
        while (opModeIsActive()) {
            if (gamepad1.y) {
                servoPosition = MID_SERVO;
            }
            if (gamepad1.x) {
                servoPosition = 0.0;
            }
            if (gamepad1.b) {
                servoPosition = 1.0;
            }
            if (gamepad1.dpad_up && !dpadUpPressed) {
                servoPosition = Math.min(servoPosition + increment, 1.0);
                dpadUpPressed = true;
            } else if (!gamepad1.dpad_up) {
                dpadUpPressed = false;
            }

            if (gamepad1.dpad_down && !dpadDownPressed) {
                servoPosition = Math.max(servoPosition - increment, 0.0);
                dpadDownPressed = true;
            } else if (!gamepad1.dpad_down) {
                dpadDownPressed = false;
            }

            if (gamepad1.dpad_right && !dpadRightPressed) {
                increment = Math.max(increment / 10, 0.001); // Decrease order of magnitude
                dpadRightPressed = true;
            } else if (!gamepad1.dpad_right) {
                dpadRightPressed = false;
            }

            if (gamepad1.dpad_left && !dpadLeftPressed) {
                increment = Math.min(increment * 10, 0.1); // Increase order of magnitude
                dpadLeftPressed = true;
            } else if (!gamepad1.dpad_left) {
                dpadLeftPressed = false;
            }

            servo.setPosition(servoPosition);
            telemetry.addData("Instructions", "Use D-pad up/down to change by " + increment + ": Use buttons X=0, Y=0.5, B=1.0: Use left/right to change order of magnitude.");
            telemetry.addData("Servo Position", servoPosition);
            telemetry.update();
        }
    }
}
