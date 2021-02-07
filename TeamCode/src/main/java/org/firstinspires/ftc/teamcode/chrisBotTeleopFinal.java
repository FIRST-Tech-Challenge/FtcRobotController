package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

@TeleOp(name="chrisBotTeleopFINAL", group="chrisBot")
//@Disabled

public class chrisBotTeleopFinal extends OpMode{

    chrisBot robot = new chrisBot();

    ElapsedTime count = new ElapsedTime();
    final long inchTime = 200;
    final double inchPower = 0.3;
    int shooterState = 0;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);
        double time =  System.currentTimeMillis();
        telemetry.clear();
        telemetry.setAutoClear(true);
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
            double robotAngle = Math.atan2(gamepadState[1], gamepadState[0]) - Math.PI / 4;
            double rightX = gamepadState[2];
            robot.setDrivePower(-1*r * Math.cos(robotAngle) + rightX, -1*r * Math.sin(robotAngle) - rightX, -1*r * Math.sin(robotAngle) + rightX, -1*r * Math.cos(robotAngle) - rightX);
        }
        else {
            robot.setAllDrivePower(0);
        }

        // Inch
        double[] powers = {};
        if(gamepad1.dpad_up) {
            powers = new double[]{inchPower,inchPower,inchPower,inchPower};
        }
        if(gamepad1.dpad_down) {
            powers = new double[]{-1*inchPower,-1*inchPower,-1*inchPower,-1*inchPower};
        }
        if(gamepad1.dpad_left) {
            powers = new double[]{-1*inchPower,inchPower,inchPower,-1*inchPower};
        }
        if(gamepad1.dpad_right) {
            powers = new double[]{inchPower,-1*inchPower,-1*inchPower,inchPower};
        }
        count.reset();
        if(!Arrays.equals(powers, new double[]{})) {
            do {
                robot.setDrivePower(powers);
            } while (count.milliseconds() < inchTime);
            robot.setAllDrivePower(0);
        }

        // Attachment code
        if(gamepad1.x) {
            telemetry.addLine("Intake speed fast");
            robot.shootOn();
        } else if (gamepad1.b) {
            telemetry.addLine("Intake speed slow");
            robot.shootOnSlow();
        }
        telemetry.update();
        if(gamepad1.a) {
            robot.intakeOn();
        } else if(gamepad1.y) {
            robot.intakeReverse();
        } else {
            robot.intakeOff();
        }

    }

    @Override
    public void stop() {
        robot.shootOff();
        robot.setAllDrivePower(0);
        super.stop();
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