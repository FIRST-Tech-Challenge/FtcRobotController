package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private Robot robot;
    private Gamepad gamepad;
    private int pos_open = 1;
    private int pos_close = 0;
    public Claw(Robot robot, Gamepad gamepad)
    {
        this.robot = robot;
        this.gamepad = gamepad;
    }

    public void open(Servo s) {
        s.setPosition(pos_open);
    }
    public void close(Servo s) {
        s.setPosition(pos_close);
    }

    public void operate()
    {
        if (gamepad.left_trigger > 0) {
            open(robot.servoCL);
        } else {
            close(robot.servoCL);
        }
        if (gamepad.right_trigger > 0) {
            open(robot.servoCR);
        } else {
            close(robot.servoCR);
        }
    }
}
