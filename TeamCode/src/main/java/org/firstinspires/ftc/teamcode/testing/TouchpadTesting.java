package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Touchpad Testing")
public class TouchpadTesting extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        float touchpad1x = 0;
        float touchpad1y = 0;
        float touchpad2x = 0;
        float touchpad2y = 0;
        if (gamepad1.touchpad_finger_1) {
            touchpad1x = gamepad1.touchpad_finger_1_x;
            touchpad1y = gamepad1.touchpad_finger_1_y;
        }

        if (gamepad1.touchpad_finger_2) {
            touchpad2x = gamepad1.touchpad_finger_2_x;
            touchpad2y = gamepad1.touchpad_finger_2_y;
        }

        telemetry.addData("Finger 1", touchpad1x + ", " + touchpad1y);
        telemetry.addData("Finger 2", touchpad2x + ", " + touchpad2y);
    }
}
