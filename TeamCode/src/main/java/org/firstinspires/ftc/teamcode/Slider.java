package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Slider {
    private Robot robot;
    private Gamepad gamepad;
    private double normal_speed = 0.8;
    private double slow_speed = 0.3;
    public Slider(Robot robot, Gamepad gamepad)
    {
        this.robot = robot;
        this.gamepad = gamepad;
    }

    private void moveOp(double power, double speed_factor)
    {
        robot.motorSlider.setPower(power * speed_factor);
    }

    public void move()
    {
        if (gamepad.left_stick_y != 0) {
            moveOp(gamepad.left_stick_y, normal_speed);
        } else if (gamepad.left_stick_x != 0) {
            moveOp(gamepad.left_stick_x, slow_speed);
        }
    }

}
