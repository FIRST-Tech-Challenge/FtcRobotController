package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="chrisBotTeleopWithIntake", group="chrisBot")
//@Disabled

public class chrisBotTeleopWithIntake extends OpMode{

    chrisBot robot = new chrisBot();

    @Override
    public void init() {

        robot.init(hardwareMap);
        double time =  System.currentTimeMillis();

    }

    @Override
    public void loop() {
        // Driving code
        if(notInDeadzone(gamepad1, "left") || notInDeadzone(gamepad1, "right")) {
            // Algorithm taken from https://ftcforum.firstinspires.org/forum/ftc-technology/android-studio/6361-mecanum-wheels-drive-code-example, quote to dmssargent
            double[] gamepadState = getGamepadState(gamepad1);
            double r = Math.hypot(gamepadState[0], gamepadState[1]);
            double robotAngle = Math.atan2(gamepadState[1], gamepadState[0]) - Math.PI / 4;
            double rightX = gamepadState[2];
            robot.setPower(r * Math.cos(robotAngle) + rightX, r * Math.sin(robotAngle) - rightX, r * Math.sin(robotAngle) + rightX, r * Math.cos(robotAngle) - rightX);
        }
        else {
            robot.setAllPower(0);
        }

        // Intake code
        if (gamepad1.x) {
            robot.intakeOn();
        } else {
            robot.intakeOff();
        }
    }

//--------------------------------- FUNCTIONS ----------------------------------------------------
    public static boolean notInDeadzone(Gamepad gamepad, String stick) {
        if (stick.equals("left")) {
            return Math.abs(gamepad.left_stick_x) > 0.1 || Math.abs(gamepad.left_stick_y) > 0.1;
        }
        else if (stick.equals("right")) {
            return Math.abs(gamepad.right_stick_x) > 0.1 || Math.abs(gamepad.right_stick_y) > 0.1;
        }
        return false;
    }

    public static double[] getGamepadState(Gamepad gamepad) {
        double[] gamepadState = {(double)gamepad.left_stick_x, (double)gamepad.left_stick_y, (double)gamepad.right_stick_x, (double)gamepad.right_stick_y};
        for (int i = 0; i < gamepadState.length; i++) {
            if (gamepadState[i] > 0.9) {
                gamepadState[i] = Math.signum(gamepadState[i])*1;
            } else if (Math.abs(gamepadState[i]) > 0.1) {
                gamepadState[i] = gamepadState[i];
            } else {
                gamepadState[i] = 0;
            }
        }
        return gamepadState;
    }
}