package com.kalipsorobotics.PID;

import android.annotation.SuppressLint;

import com.kalipsorobotics.math.Position;
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

        PIDController activeController = driveTrain.xController;
        Gamepad prev = new Gamepad();
        Gamepad current = new Gamepad();

        double xMovement = 6;
        double yMovement = 0;
        double hMovement = 0;

        while (opModeIsActive()) {
            prev.copy(current);
            current.copy(gamepad1);

            if (current.dpad_left && !prev.dpad_left) {
                xMovement -= 1;
            }
            if (current.dpad_right && !prev.dpad_right) {
                xMovement += 1;
            }
            if (current.dpad_up && !prev.dpad_up) {
                yMovement += 1;
            }
            if (current.dpad_down && !prev.dpad_down) {
                yMovement -= 1;
            }
            if (current.right_bumper && !prev.right_bumper) {
                hMovement += current.left_bumper ? -10 : 10;
            }

            if (current.x && !prev.x) {
                activeController.chKp(current.left_bumper ? -0.001 : 0.001);
            }
            if (current.y && !prev.y) {
                activeController.chKi(current.left_bumper ? -0.0005 : 0.0005);
            }
            if (current.a && !prev.a) {
                activeController.chKd(current.left_bumper ? -0.0005 : 0.0005);
            }
            if (current.b && !prev.b) {
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

            if (gamepad1.start && !prev.start) {
                driveTrain.xController.reset();
                driveTrain.yController.reset();
                driveTrain.headingController.reset();

                driveTrain.odometryFuse.wheelResetData();
                driveTrain.odometryFuse.sparkResetData(true, 0);
                System.out.println("Reset PID Controller");

                driveTrain.move(xMovement, yMovement, hMovement, telemetry);
            }


            Position pos = driveTrain.odometryFuse.pointCollectData();
            double heading = driveTrain.odometryFuse.headingUpdateData("right", 0, 0);
            telemetry.addLine(String.format("x | currently at %f", pos.getX()));
            telemetry.addLine(String.format("y | currently at %f", pos.getY()));
            telemetry.addLine(String.format("h | currently at %f", heading));
            telemetry.update();
        }

    }
}
