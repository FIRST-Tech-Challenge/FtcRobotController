package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Arm {
    private Robot robot;
    private Gamepad gamepad;
    static private double pos_pixel  = 0.65;
    static private double pos_folded  = 0.0;
    static private double pos_backdrop  = 0.4;
    public Arm(Robot robot, Gamepad gamepad) {
        this.robot = robot;
        this.gamepad = gamepad;
    }

    public void arm_pixel()
    {
        robot.servoArm.setPosition(pos_pixel);
    }

    public void arm_fold()
    {
        robot.servoArm.setPosition(pos_folded);
    }

    public void arm_backdrop() {
        robot.servoArm.setPosition(pos_backdrop);
    }
    public void operate()
    {
        if (gamepad.x) {
            arm_pixel();
        } else if (gamepad.y) {
            arm_backdrop();
        } else if (gamepad.b) {
            arm_fold();
        } else if (gamepad.right_stick_y != 0) {
            //robot.servoArm.setPosition((1.0 - gamepad.right_stick_y ) % 1.0);
        }
    }
}
