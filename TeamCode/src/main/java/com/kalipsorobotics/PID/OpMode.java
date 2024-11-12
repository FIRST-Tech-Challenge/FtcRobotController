package com.kalipsorobotics.PID;

import android.annotation.SuppressLint;

import com.kalipsorobotics.math.Point;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class OpMode extends LinearOpMode {
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        TestingDriveTrain driveTrain = new TestingDriveTrain(hardwareMap);
        waitForStart();

        boolean flag = false;
        PIDController activeController = driveTrain.xController;
        Gamepad current = new Gamepad();
        Gamepad prev = new Gamepad();

        double xMovement = 6;
        double yMovement = 0;
        double hMovement = 0;

        while (opModeIsActive()) {
            prev.copy(current);
            current.copy(gamepad1);

            if (current.dpad_left && !prev.dpad_left) {
                xMovement -= 1;
            }
            if (gamepad1.dpad_right && !prev.dpad_right) {
                xMovement += 1;
            }
            if (gamepad1.dpad_up && !prev.dpad_up) {
                yMovement += 1;
            }
            if (gamepad1.dpad_down && !prev.dpad_down) {
                yMovement -= 1;
            }
            if (gamepad1.left_trigger > 0.5 && prev.left_trigger == 0) {
                hMovement += 5;
            }
            if (gamepad1.right_trigger > 0.5 && prev.right_trigger == 0) {
                hMovement -= 5;
            }

            if (gamepad1.x && !prev.x) {
                activeController.chKp(0.001);
            }
            if (gamepad1.y && !prev.y) {
                activeController.chKi(0.0005);
            }
            if (gamepad1.a && !prev.a) {
                activeController.chKd(0.001);
            }
            if (gamepad1.b && !prev.b) {
                PIDController newController = null;
                if (activeController == driveTrain.xController) {
                    newController = driveTrain.yController;
                }
                if (activeController == driveTrain.yController) {
                    newController = driveTrain.headingController;
                }
                if (activeController == driveTrain.headingController) {
                    newController = driveTrain.xController;
                }
                activeController = newController;
            }

            telemetry.addLine(String.format("Using %s", activeController));
            telemetry.addLine(String.format("Moving X by %f, Y by %f, H by %f", xMovement, yMovement, hMovement));

            if (gamepad1.start && !flag) {
                driveTrain.move(xMovement, yMovement, hMovement, telemetry);
                flag = true;
            }

            if (gamepad1.back) {
                flag = false;
                activeController.reset();
                System.out.println("Reset PID Controller");
            }

            Point pos = driveTrain.odometryFuse.PointCollectData();
            double heading = driveTrain.otos.getPosition().h;
            telemetry.addLine(String.format("x | currently at %f", pos.getX()));
            telemetry.addLine(String.format("y | currently at %f", pos.getY()));
            telemetry.addLine(String.format("h | currently at %f", heading));
            telemetry.update();
        }

    }
}
