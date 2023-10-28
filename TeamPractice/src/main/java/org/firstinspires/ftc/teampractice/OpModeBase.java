package org.firstinspires.ftc.teampractice;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class OpModeBase extends LinearOpMode {
    final protected Robot robot = new Robot();
    public volatile boolean a_pressed, b_pressed, x_pressed, y_pressed, lb_pressed, rb_pressed, dpad_pressed, back_pressed, guide_pressed, start_pressed;
    public volatile float left_trigger, right_trigger;
    public final void gamepadUpdate() {
        a_pressed = gamepad1.a || gamepad2.a;
        b_pressed = gamepad1.b || gamepad2.b;
        x_pressed = gamepad1.x || gamepad2.x;
        y_pressed = gamepad1.y || gamepad2.y;
        lb_pressed = gamepad1.left_bumper || gamepad2.left_bumper;
        rb_pressed = gamepad1.right_bumper || gamepad2.right_bumper;
        dpad_pressed = gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_right ||
                gamepad2.dpad_down || gamepad2.dpad_up || gamepad2.dpad_left || gamepad2.dpad_right;
        start_pressed = gamepad1.start;
        left_trigger = gamepad1.left_trigger;
        right_trigger = gamepad1.right_trigger;
    }
}
