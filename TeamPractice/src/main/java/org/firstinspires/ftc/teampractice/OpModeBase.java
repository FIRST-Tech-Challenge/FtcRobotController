package org.firstinspires.ftc.teampractice;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class OpModeBase extends LinearOpMode {
    final protected Robot robot = new Robot();
    public volatile boolean a_pressed, b_pressed, x_pressed, y_pressed, lb_pressed, rb_pressed, dpad_pressed, back_pressed, guide_pressed, start_pressed;
    public volatile float left_trigger, right_trigger;
}
