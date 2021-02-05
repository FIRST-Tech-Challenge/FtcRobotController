package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@TeleOp(name="chrisBotTeleopFINAL", group="chrisBot")
//@Disabled

public class chrisBotTeleopFinal extends OpMode{

    chrisBot robot = new chrisBot();

    ElapsedTime count = new ElapsedTime();
    long t = System.currentTimeMillis();
    boolean holding, inching;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);
        double time =  System.currentTimeMillis();
    }

    @Override
    public void loop() {
        // Driving code
        if(notInDeadzone(gamepad1, "left") || notInDeadzone(gamepad1, "right")) {
            // Algorithm taken from https://ftcforum.firstinspires.org/forum/ftc-technology/android-studio/6361-mecanum-wheels-drive-code-example, quote to dmssargent
            double[] gamepadState = getGamepadState(gamepad1);
            double r = Math.hypot(gamepadState[0], gamepadState[1]);
            if (gamepad1.right_bumper) {
                r=0.4*r;
            }
            double robotAngle = -1*(Math.atan2(gamepadState[1], gamepadState[0]) - Math.PI / 4);
            double rightX = gamepadState[2];
            robot.setPower(-1*r * Math.cos(robotAngle) + rightX, -1*r * Math.sin(robotAngle) - rightX, -1*r * Math.sin(robotAngle) + rightX, -1*r * Math.cos(robotAngle) - rightX);
        }
        else {
            robot.setAllPower(0);
        }

        // Inch
        if(!holding) {
            double[] powers = {};
            if(gamepad1.dpad_up) {
                powers = new double[]{0.1,0.1,0.1,0.1};
            }
            if(gamepad1.dpad_down) {
                powers = new double[]{-0.1,-0.1,-0.1,-0.1};
            }
            if(gamepad1.dpad_left) {
                powers = new double[]{0.1,-0.1,-0.1,0.1};
            }
            if(gamepad1.dpad_right) {
                powers = new double[]{-0.1,0.1,0.1,-0.1};
            }
            if (!(Arrays.equals(new double[]{}, powers))) {
                holding = true;
                count.reset();
                t = System.currentTimeMillis();
                robot.setPower(powers);
            }

        }
        if (count.milliseconds() - t > 300) {
            robot.setAllPower(0);
            if (!(gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left)) {
                holding = false;
            }
        }

        // Attachment code
        if(gamepad1.x) {
            robot.shootOn(0.6);
        } else if (gamepad1.b) {
            robot.shootOn(0.4);
        } else {
            robot.shootOff();
        } if(gamepad1.a) {
            robot.intakeOn();
        } else if(gamepad1.y) {
            robot.intakeReverse();
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