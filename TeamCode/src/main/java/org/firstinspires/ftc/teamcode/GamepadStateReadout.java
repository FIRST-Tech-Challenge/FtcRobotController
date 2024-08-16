package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Gamepad Attributes Example")
public class GamepadStateReadout extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Digital Buttons
            telemetry.addData("A Button", gamepad1.a);
            telemetry.addData("B Button", gamepad1.b);
            telemetry.addData("X Button", gamepad1.x);
            telemetry.addData("Y Button", gamepad1.y);
            telemetry.addData("Dpad Up", gamepad1.dpad_up);
            telemetry.addData("Dpad Down", gamepad1.dpad_down);
            telemetry.addData("Dpad Left", gamepad1.dpad_left);
            telemetry.addData("Dpad Right", gamepad1.dpad_right);
            telemetry.addData("Left Bumper", gamepad1.left_bumper);
            telemetry.addData("Right Bumper", gamepad1.right_bumper);
            telemetry.addData("Left Stick Button", gamepad1.left_stick_button);
            telemetry.addData("Right Stick Button", gamepad1.right_stick_button);
            telemetry.addData("Start Button", gamepad1.start);
            telemetry.addData("Back Button", gamepad1.back);
            telemetry.addData("Guide Button", gamepad1.guide);

            // Analog Inputs
            telemetry.addData("Left Stick X", gamepad1.left_stick_x);
            telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
            telemetry.addData("Right Stick X", gamepad1.right_stick_x);
            telemetry.addData("Right Stick Y", gamepad1.right_stick_y);
            telemetry.addData("Left Trigger", gamepad1.left_trigger);
            telemetry.addData("Right Trigger", gamepad1.right_trigger);

            // Other Attributes
            telemetry.addData("Timestamp", gamepad1.timestamp);
            telemetry.addData("User", gamepad1.getUser());
            telemetry.addData("ID", gamepad1.id);
            telemetry.addData("Type", gamepad1.type);

            telemetry.update();
            sleep(20);
        }
    }
}
