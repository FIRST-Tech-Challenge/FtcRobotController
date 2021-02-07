package org.firstinspires.ftc.teamcode.disabled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="testPlatfromTeleopV2", group="Zippo")
@Disabled

public class testPlatformTeleopV2 extends OpMode{

    testPlatformHardware robot  = new testPlatformHardware();

    // Create variables for motor power
    private double[] powers = new double[4]; // front left, front right, back left, back right

    @Override
    public void init() {

        robot.init(hardwareMap);
        double time =  System.currentTimeMillis();

    }

    @Override
    public void loop() {
        if(notInDeadzone(gamepad1, "left") || notInDeadzone(gamepad1, "right")) {
            // Algorithm taken from https://ftcforum.firstinspires.org/forum/ftc-technology/android-studio/6361-mecanum-wheels-drive-code-example, quote to dmssargent
            double[] gamepadState = getGamepadState(gamepad1);
            double r = Math.hypot(gamepadState[0], gamepadState[1]);
            double robotAngle = Math.atan2(gamepadState[1], gamepadState[0]) - Math.PI / 4;
            double rightX = gamepadState[2];
            setPower(powers,r * Math.cos(robotAngle) + rightX, r * Math.sin(robotAngle) - rightX, r * Math.sin(robotAngle) + rightX, r * Math.cos(robotAngle) - rightX);
        }
        else {
            setPower(powers,0,0,0,0);
        }
        pushPower();
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

    public static void setPower(double[] powers, double flPower, double frPower, double blPower, double brPower)
    {
        powers[0] = flPower;
        powers[1] = frPower;
        powers[2] = blPower;
        powers[3] = brPower;
    }
    public void pushPower() {
        for (int i = 0; i < powers.length; i++) {
            if(powers[i] > 1) {
                powers[i] = 1;
            }
            else if(powers[i] < -1) {
                powers[i] = -1;
            }
        }
        robot.motorFrontLeft.setPower(powers[0]);
        robot.motorFrontRight.setPower(powers[1]);
        robot.motorBackLeft.setPower(powers[2]);
        robot.motorBackRight.setPower(powers[3]);
    }
}