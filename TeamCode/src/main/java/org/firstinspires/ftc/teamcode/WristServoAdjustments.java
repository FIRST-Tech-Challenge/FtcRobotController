package org.firstinspires.ftc.teamcode;

// All the things that we use and borrow

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Adjustments", group="Linear OpMode")
public class WristServoAdjustments extends LinearOpMode {

    // This chunk controls our claw
    Servo claw = null;
    double claw_position = 0;

    Servo wrist = null;
    double wrist_position = 0;

    Servo ascentStick = null;
    double ascentStick_position = 0;

    @Override
    //Op mode runs when the robot runs. It runs the whole time.
    public void runOpMode() {

        initializeHardwareVariables();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Servo Tuner", "press PLAY");
        telemetry.update();
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // Control the claw
            if (gamepad1.right_bumper) {
                claw_position += 0.001;
            }
            if (gamepad1.left_bumper) {
                claw_position -= 0.001;
            }
            claw.setPosition(claw_position);

            // Control the wrist
            if (gamepad1.right_trigger > 0) {
                wrist_position += 0.001;
            }
            if (gamepad1.left_trigger > 0) {
                wrist_position -= 0.001;
            }
            wrist.setPosition(wrist_position);

            // Control the ascent stick
            if (gamepad1.dpad_up) {
                ascentStick_position += 0.001;
            }

            if (gamepad1.dpad_down) {
                ascentStick_position -= 0.001;
            }
            ascentStick.setPosition(ascentStick_position);
            printDataOnScreen();
        }
    }

    private void initializeHardwareVariables() {
        claw = hardwareMap.get(Servo.class, "claw");

        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.setDirection(Servo.Direction.REVERSE);
        ascentStick = hardwareMap.get(Servo.class, "ascentStick");
        ascentStick.setDirection(Servo.Direction.REVERSE);
    }


    // Log all (relevant) info about the robot on the hub.
    private void printDataOnScreen() {
        telemetry.addData("Target claw position", "%4.2f", claw_position);
        telemetry.addData("Claw position", "%4.2f", claw.getPosition());
        telemetry.addData("Target wrist position", "%4.2f", wrist_position);
        telemetry.addData("Wrist position", "%4.2f", wrist.getPosition());
        telemetry.addData("Target ascentStick position", "%4.2f", ascentStick_position);
        telemetry.addData("AscentStick position", "%4.2f", ascentStick.getPosition());

        telemetry.update();
    }
}