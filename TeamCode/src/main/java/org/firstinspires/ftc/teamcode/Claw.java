package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Claw {
    private Robot robot;
    private Gamepad gamepad;
    public Claw(Robot robot, Gamepad gamepad)
    {
        this.robot = robot;
        this.gamepad = gamepad;
    }

    public void openLeft() {
        robot.servoCL.setPosition(1);
    }
    public void closeLeft() {
        robot.servoCL.setPosition(0);
    }
    public void closeRight() {
        robot.servoCR.setPosition(0);
    }
    public void openRight() {
        robot.servoCR.setPosition(1);
    }

    public void operate()
    {
        if (gamepad.a) {
            closeLeft();
        } else if (gamepad.b) {
            openLeft();
        } else if (gamepad.x) {
            closeRight();
        } else if (gamepad.y) {
            openRight();
        }
    }
}
