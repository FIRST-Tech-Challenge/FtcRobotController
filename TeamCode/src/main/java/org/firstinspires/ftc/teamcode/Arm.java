package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Arm {
    private Robot robot;
    private Gamepad gamepad;
    private double normal_speed = 0.8;
    private double slow_speed = 0.2;
    public Arm(Robot robot, Gamepad gamepad) {
        this.robot = robot;
        this.gamepad = gamepad;
    }

    private void moveOp(double power, double speed_factor)
    {
        robot.motorArm.setPower(power * speed_factor);
    }

    public void move()
    {
        if (gamepad.right_stick_y != 0) {
            moveOp(gamepad.right_stick_y, normal_speed);
        } else if (gamepad.right_stick_x != 0) {
            moveOp(gamepad.right_stick_x, slow_speed);
        }
    }
}
