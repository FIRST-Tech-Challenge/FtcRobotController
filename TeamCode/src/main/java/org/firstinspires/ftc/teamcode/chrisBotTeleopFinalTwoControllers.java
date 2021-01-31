package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="chrisBotTeleopFINAL2C", group="chrisBot")
//@Disabled

public class chrisBotTeleopFinalTwoControllers extends OpMode{

    chrisBot robot = new chrisBot();

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);
        double time =  System.currentTimeMillis();
    }

    @Override
    public void loop() {
        // Driving code
        // Algorithm taken from https://ftcforum.firstinspires.org/forum/ftc-technology/android-studio/6361-mecanum-wheels-drive-code-example, quote to dmssargent
        double r = 0, robotAngle = 0, rightX = 0;
        boolean driving = false;
        double[] gamepadState1 = getGamepadState(gamepad1), gamepadState2 = getGamepadState(gamepad2);
        if(notInDeadzone(gamepad1, "left")) {
            driving = true;
            r = Math.hypot(gamepadState1[0], gamepadState1[1]);
            robotAngle = Math.atan2(gamepadState1[1], gamepadState1[0]) - Math.PI / 4;
        }
        if(!notInDeadzone(gamepad1, "left") && notInDeadzone(gamepad2, "left")) {
            driving = true;
            r = Math.hypot(gamepadState2[0], gamepadState2[1]);
            robotAngle = Math.atan2(gamepadState2[1], gamepadState2[0]) - Math.PI / 4;
        }
        if(notInDeadzone(gamepad1, "right")) {
            driving = true;
            rightX = gamepadState1[2];
        }
        if(!notInDeadzone(gamepad1, "right") && notInDeadzone(gamepad2, "right")) {
            driving = true;
            rightX = gamepadState2[2];
        }
        if(driving) {
            robot.setPower(r * Math.cos(robotAngle) + rightX, r * Math.sin(robotAngle) - rightX, r * Math.sin(robotAngle) + rightX, r * Math.cos(robotAngle) - rightX);
        }
        else {
            robot.setAllPower(0);
        }

        // Attachment code
        if (gamepad1.b || (gamepad1.a && gamepad1.x) || gamepad2.b || (gamepad2.a && gamepad2.x)) {
            robot.shootOn();
            robot.intakeOn();
        } else if (gamepad1.x || gamepad2.x) {
            robot.shootOn();
            robot.intakeOff();
        } else if (gamepad1.a || gamepad2.a) {
            robot.intakeOn();
            robot.shootOff();
        } else {
            robot.shootOff();
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