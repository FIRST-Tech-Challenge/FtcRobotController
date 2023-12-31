package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Full", group = "CenterStage")
public class CSFullTeleOp extends CSMethods {

    // Declare OpMode members for each of the 4 motors.
    boolean lBack = false;
    boolean rBack = false;
    boolean a = false;
    double axial = 0.0;
    double lateral = 0.0;
    double yaw = 0.0;
    final double speedMultiplier = 0.75;
    final double baseTurnSpeed = 2.5;
    double slowdownMultiplier = 0.0;


    @Override
    public void runOpMode() {
        setup(true);

        double carWashPower = 1.0;

        while (opModeIsActive()) {
            slowdownMultiplier = (1.0 - gamepad1.right_trigger);

            axial = ((-gamepad1.left_stick_y * speedMultiplier) * slowdownMultiplier);
            lateral = ((gamepad1.left_stick_x * speedMultiplier) * slowdownMultiplier);
            yaw = ((gamepad1.right_stick_x * baseTurnSpeed) * slowdownMultiplier);

            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send calculated power to wheels
            lf.setPower(leftFrontPower);
            rf.setPower(rightFrontPower);
            lb.setPower(leftBackPower);
            rb.setPower(rightBackPower);

            if (pixelLiftingMotor != null) {
                if (gamepad2.dpad_up && !gamepad2.dpad_down) {
                    if (pixelLiftingMotor.getCurrentPosition() < 3000) {
                        pixelLiftingMotor.setPower(0.1);
                    } else {
                        pixelLiftingMotor.setPower(0);
                    }
                }


                if (gamepad2.dpad_down && !gamepad2.dpad_up) {
                    if (pixelLiftingMotor.getCurrentPosition() > 0) {
                        pixelLiftingMotor.setPower(-0.1);
                    } else {
                        pixelLiftingMotor.setPower(0);
                    }
                }


                if (!gamepad2.dpad_up && !gamepad2.dpad_down || gamepad2.dpad_down && gamepad2.dpad_up) {
                    pixelLiftingMotor.setPower(0);
                }
            }

            if (carWashMotor != null) {
                if (gamepad2.a) {
                    carWashMotor.setPower(carWashPower);
                }
                if (gamepad2.b) {
                    carWashMotor.setPower(-carWashPower);
                }
                if (!gamepad2.a && !gamepad2.b) {
                    carWashMotor.setPower(0);
                }
            }

            if (pixelBackServo != null) {
                if (gamepad2.y && !lBack) {
                    lBack = true;
                    if (pixelBackServo.getPosition() == 1) {
                        pixelBackServo.setPosition(0);
                    } else {
                        pixelBackServo.setPosition(1);
                    }
                } else if (!gamepad2.y) {
                    lBack = false;
                }
            }

            if (trayTiltingServo != null) {
                if ((gamepad2.left_trigger > 0.25) && !a) {
                    a = true;
                    if (trayTiltingServo.getPosition() <= 0.355 && trayTiltingServo.getPosition() >= 0.345) {
                        trayTiltingServo.setPosition(0);
                    } else {
                        trayTiltingServo.setPosition(0.35);
                    }
                } else if (!(gamepad2.left_trigger > 0.25)) {
                    a = false;
                }
            }

            if (pixelFrontServo != null) {
                if (gamepad2.x && !rBack) {
                    rBack = true;
                    if (pixelFrontServo.getPosition() == 1) {
                        pixelFrontServo.setPosition(0);
                    } else {
                        pixelFrontServo.setPosition(1);
                    }
                } else if (!gamepad2.x) {
                    rBack = false;
                }
            }

            if (droneServo != null) {
                if (gamepad2.left_bumper && gamepad2.right_bumper) {
                    droneServo.setPosition(1);
                }
            }

            telemetry.addData("Run Time", runtime.toString());
            if (runtime.seconds() > 90) {
                telemetry.addData("Game Phase", "End Game");
            } else {
                telemetry.addData("Game Phase", "Driver Controlled");
            }
            telemetry.update();
        }
    }
}