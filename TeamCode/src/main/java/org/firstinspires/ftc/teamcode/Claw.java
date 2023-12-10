package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private Robot robot;
    private Gamepad gamepad;
    private boolean cl_closed = true;
    private boolean cr_closed = true;
    public Claw(Robot robot, Gamepad gamepad)
    {
        this.robot = robot;
        this.gamepad = gamepad;
        robot.servoCL.setPosition(1);
        robot.servoCR.setPosition(0);
    }



    public void operate()
    {
        robot.telemetry.addData("Claw_Right: Status", cr_closed);
        robot.telemetry.addData("Claw_Left: Status", cl_closed);
        robot.telemetry.update();

        if (gamepad.right_trigger > 0.5) {
            robot.servoCL.setPosition(0); // open the claw
            cl_closed = false;
        } else {
            if (cl_closed == false) {
                robot.servoCL.setPosition(1); // close the claw
            }
            cl_closed = true;
        }
        if (gamepad.left_trigger > 0.5) {
            robot.servoCR.setPosition(1); // open the claw
            cr_closed = false;
        } else {
            if (cr_closed == false) {
                robot.servoCR.setPosition(0); // close the claw
            }
            cr_closed = true;
        }
    }
}
