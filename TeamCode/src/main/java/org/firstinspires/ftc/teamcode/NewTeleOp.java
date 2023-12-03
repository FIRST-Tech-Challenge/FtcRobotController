package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class NewTeleOp extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {

        // initialize robot class
        robot = new Robot(hardwareMap, this, telemetry, true, false);
        robot.setUpDrivetrainMotors();
        robot.setUpIntakeOuttake();

        boolean hangingMode = false;
        int polarity = 1;
        int TRIGGER_PRESSED = 0; // TODO: test

        robot.openHook();
        robot.trayToIntakePos();
        robot.moveLinearSlideByTicksBlocking(0);
        robot.closeClamp();

        waitForStart();

        while (opModeIsActive()) {

            // GAMEPAD 1: DRIVER CONTROLS

            // right bumper launches drone
            if (gamepad1.right_bumper) {
                robot.planeLauncher.setPosition(0.7);
            }

            // x aligns bot to board
            if (gamepad1.x) {
                if (robot.isRedAlliance) {
                    robot.setHeading(-90, 0.7);
                } else {
                    robot.setHeading(90, 0.7);
                }
            }


            if (gamepad1.a) {
                polarity = 1;
            } else if (gamepad1.y) {
                polarity = -1;
            }

            //doubles for amount of input for straight, turning, and mecanuming variables
            double straight;
            double turning;
            double mecanuming;

            //setting forward and mecanum based on where the front is
            straight = gamepad1.left_stick_y * polarity * -0.75;
            mecanuming = gamepad1.left_stick_x * polarity * 0.75;

            //turning stays the same
            turning = gamepad1.right_stick_x * 0.75;

            //set powers using this input
            double fLeftPower = straight + turning + mecanuming;
            double fRightPower = straight - turning - mecanuming;
            double bLeftPower = straight + turning - mecanuming;
            double bRightPower = straight - turning + mecanuming;

            //scale powers
            double maxPower = maxAbsValueDouble(fLeftPower, bLeftPower, fRightPower, bRightPower);

            if (Math.abs(maxPower) > 1) {
                double scale = Math.abs(maxPower);
                fLeftPower /= scale;
                bLeftPower /= scale;
                fRightPower /= scale;
                bRightPower /= scale;
            }

            robot.setMotorPower(fLeftPower, fRightPower, bLeftPower, bRightPower);

            // GAMEPAD 2: ARM CONTROLS

            // dpad controlling lock
            if (gamepad2.dpad_up) { // up - close
                robot.closeHook();
            } else if (gamepad2.dpad_down) { // down - open
                robot.openHook();
            }

            // pivoting tray
            if (gamepad2.a && gamepad2.y) { // both - stay at current
                // do nothing
            } else if (gamepad2.a) { // a - intake position
                robot.trayToIntakePos();
            } else if (gamepad2.y) { // y - outtake position
                robot.trayToOuttakePos();
            }

            // intake regurgitate
            if (gamepad2.left_trigger > TRIGGER_PRESSED && gamepad2.left_bumper) { // both - nothing
                // do nothing
            } else if (gamepad2.left_trigger > TRIGGER_PRESSED) { // left trigger - intake
                robot.intake.setPower(-0.7);
            } else if (gamepad2.left_bumper) { // left bumper - regurgitate
                robot.intake.setPower(0.7);
            } else { // neither - stop
                robot.intake.setPower(0);
            }

            // clamp controls
            if (gamepad2.right_trigger > TRIGGER_PRESSED) { // right trigger or trigger & bumper - close clamp
                robot.closeClamp();
            } else if (gamepad2.right_bumper) { // bumper - open clamp
                robot.openClamp();
            }

            // b - hanging mode
            if (gamepad2.b) {
                hangingMode = true;
            }

            //b to turn on hanging mode
            if (!hangingMode) {
                //if not hanging, power less
                if (-gamepad2.left_stick_y > 0) {
                    robot.lsBack.setPower(0.5);
                    robot.lsFront.setPower(0.5);
                } else if (-gamepad2.left_stick_y < 0) {
                    robot.lsBack.setPower(-0.5);
                    robot.lsFront.setPower(-0.5);
                } else {
                    robot.lsBack.setPower(0);
                    robot.lsFront.setPower(0);
                }
            } else if (hangingMode) {
                //if hanging, power more
                if (-gamepad2.left_stick_y > 0) {
                    robot.lsBack.setPower(1);
                    robot.lsFront.setPower(1);
                } else if (-gamepad2.left_stick_y < 0) {
                    robot.lsBack.setPower(-1);
                    robot.lsFront.setPower(-1);
                } else {
                    robot.lsBack.setPower(0);
                    robot.lsFront.setPower(0);
                }

            }
        }
    }

    private double maxAbsValueDouble(double a, double... others) {
        double max = a;

        for (double next : others) {
            if (Math.abs(next) > Math.abs(max)) {
                max = next;
            }
        }

        return max;
    }

}
