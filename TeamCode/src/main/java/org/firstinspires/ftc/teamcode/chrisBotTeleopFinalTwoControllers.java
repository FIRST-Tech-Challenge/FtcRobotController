package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@TeleOp(name="chrisBotTeleopFINALtwoControllers", group="chrisBot")
//@Disabled

public class chrisBotTeleopFinalTwoControllers extends OpMode{

    chrisBot robot = new chrisBot();

    ElapsedTime count = new ElapsedTime();
    final long inchTime = 200;
    final double inchPower = 0.3;
    ElapsedTime count2 = new ElapsedTime();
    boolean forward = true;
    boolean shooterState = false;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);
        double time =  System.currentTimeMillis();
        telemetry.clear();
        telemetry.setAutoClear(true);
    }

    @Override
    public void loop() {
        // Drive/inch
        if(notInDeadzone(gamepad2, "left") || notInDeadzone(gamepad2, "right")) {
            // Algorithm taken from https://ftcforum.firstinspires.org/forum/ftc-technology/android-studio/6361-mecanum-wheels-drive-code-example, quote to dmssargent
            double[] gamepadState = getGamepadState(gamepad2);
            double r = Math.hypot(gamepadState[0], gamepadState[1]);
            if (gamepad1.right_bumper) {
                r=0.4*r;
            }
            double robotAngle = Math.atan2(gamepadState[1], gamepadState[0]) - Math.PI / 4;
            double rightX = gamepadState[2];
            robot.setDrivePower(-1*r * Math.cos(robotAngle) + rightX, -1*r * Math.sin(robotAngle) - rightX, -1*r * Math.sin(robotAngle) + rightX, -1*r * Math.cos(robotAngle) - rightX);
        }
        else if(gamepad2.dpad_left || gamepad2.dpad_up || gamepad2.dpad_right || gamepad2.dpad_down) {
            double[] powers = {};
            if(gamepad2.dpad_up) {
                powers = new double[]{inchPower,inchPower,inchPower,inchPower};
                telemetry.addLine("Slow Drive: Up");
            }
            if(gamepad2.dpad_down) {
                powers = new double[]{-1*inchPower,-1*inchPower,-1*inchPower,-1*inchPower};
                telemetry.addLine("Slow Drive: Down");
            }
            if(gamepad2.dpad_left) {
                powers = new double[]{-1*inchPower,inchPower,inchPower,-1*inchPower};
                telemetry.addLine("Slow Drive: Strafe Left");
            }
            if(gamepad2.dpad_right) {
                powers = new double[]{inchPower,-1*inchPower,-1*inchPower,inchPower};
                telemetry.addLine("Slow Drive: Strafe Right");
            }
            count.reset();
            if(!Arrays.equals(powers, new double[]{})) {
                do {
                    robot.setDrivePower(powers);
                } while (count.milliseconds() < inchTime);
                robot.setAllDrivePower(0);
            }
        }
        else if(gamepad1.left_bumper) {
            if(count2.milliseconds() > 100) {
                forward = !forward;
                count2.reset();
            }
            if(forward) {
                robot.setAllDrivePower(chrisBotConstants.JIGGLE_SPEED);
            } else {
                robot.setAllDrivePower(-1*chrisBotConstants.JIGGLE_SPEED);
            }
        }
        else if(gamepad1.dpad_left || gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_down) {
            double[] powers = {};
            if(gamepad1.dpad_up) {
                powers = new double[]{inchPower,inchPower,inchPower,inchPower};
                telemetry.addLine("Slow Drive: Up");
            }
            if(gamepad1.dpad_down) {
                powers = new double[]{-1*inchPower,-1*inchPower,-1*inchPower,-1*inchPower};
                telemetry.addLine("Slow Drive: Down");
            }
            if(gamepad1.dpad_left) {
                powers = new double[]{-1*inchPower,inchPower,inchPower,-1*inchPower};
                telemetry.addLine("Slow Drive: Strafe Left");
            }
            if(gamepad1.dpad_right) {
                powers = new double[]{inchPower,-1*inchPower,-1*inchPower,inchPower};
                telemetry.addLine("Slow Drive: Strafe Right");
            }
            count.reset();
            if(!Arrays.equals(powers, new double[]{})) {
                do {
                    robot.setDrivePower(powers);
                } while (count.milliseconds() < inchTime);
                robot.setAllDrivePower(0);
            }
        } else {
            if(notInDeadzone(gamepad1, "left") || notInDeadzone(gamepad1, "right")) {
                // Algorithm taken from https://ftcforum.firstinspires.org/forum/ftc-technology/android-studio/6361-mecanum-wheels-drive-code-example, quote to dmssargent
                double[] gamepadState = getGamepadState(gamepad1);
                double r = Math.hypot(gamepadState[0], gamepadState[1]);
                if (gamepad1.right_bumper) {
                    r=0.4*r;
                }
                double robotAngle = Math.atan2(gamepadState[1], gamepadState[0]) - Math.PI / 4;
                double rightX = gamepadState[2];
                robot.setDrivePower(r * Math.cos(robotAngle) + rightX, r * Math.sin(robotAngle) - rightX, r * Math.sin(robotAngle) + rightX, r * Math.cos(robotAngle) - rightX);
            }
            else {
                robot.setAllDrivePower(0);
            }
        }

        // Attachment code
        if(gamepad1.x && !shooterState) {
            shooterState = true;
            telemetry.addLine("Shooter speed fast");
            robot.shootOn();
        } else if (gamepad1.x) {
            shooterState = false;
            telemetry.addLine("Shooter speed slow");
            robot.shootOnSlow();
        }

        telemetry.update();
        if(gamepad1.a) {
            robot.intakeOn();
            telemetry.addLine("Intake: Feed");
        } else if(gamepad1.y) {
            robot.intakeBottom();
            telemetry.addLine("Intake: Reverse");
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
        double[] gamepadState = {-1*(double)gamepad.left_stick_x, (double)gamepad.left_stick_y, (double)gamepad.right_stick_x, (double)gamepad.right_stick_y};
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