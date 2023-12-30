package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private Robot robot;
    private Gamepad gamepad;
    private boolean cl_closed = false;
    private boolean cr_closed = false;

    public Claw(Robot robot, Gamepad gamepad) {
        this.robot = robot;
        this.gamepad = gamepad;
        close_claw_left();
        close_claw_right();
    }

    public void open_claw_right()
    {
        if(cr_closed) {
            robot.servoCL.setPosition(0); // open the claw
            cr_closed = false;
        }
    }

    public void open_claw_left()
    {
        if (cl_closed) {
            robot.servoCR.setPosition(1); // open the claw
            cl_closed = false;
        }
    }

    public void close_claw_left()
    {
        if (!cl_closed) {
            robot.servoCR.setPosition(0.75); // close the claw
            cl_closed = true;
        }
    }

    public void close_claw_right()
    {
        if (!cr_closed) {
            robot.servoCL.setPosition(0.3); // close the claw
            cr_closed = true;
        }
    }

    public void operate() {
        if (gamepad.right_trigger > 0.9) {
            open_claw_right();
        } else {
            close_claw_right();
        }
        if (gamepad.left_trigger > 0.9) {
            open_claw_left();
        } else {
            close_claw_left();
        }
    }
}
