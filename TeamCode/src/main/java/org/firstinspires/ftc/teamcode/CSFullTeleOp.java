package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Full", group = "CenterStage")
public class CSFullTeleOp extends CSMethods {

    // Declare OpMode members for each of the 4 motors.
    boolean wasY = false;
    boolean wasX = false;
    boolean wasLT = false;
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
                if (!gamepad2.dpad_up && !gamepad2.dpad_down || gamepad2.dpad_down && gamepad2.dpad_up) {
                    pixelLiftingMotor.setPower(0);
                } else {
                    if (gamepad2.dpad_up && !gamepad2.dpad_down) {
                        if (pixelLiftingMotor.getCurrentPosition() < 3000) {
                            pixelLiftingMotor.setPower(0.75);
                            addActTelemetry("pixelLiftingMotor now moving");
                        } else {
                            pixelLiftingMotor.setPower(0);
                            addActTelemetry("pixelLiftingMotor no longer moving");
                        }
                    }
                    if (gamepad2.dpad_down && !gamepad2.dpad_up) {
                        if (pixelLiftingMotor.getCurrentPosition() > 0) {
                            pixelLiftingMotor.setPower(-0.75);
                            addActTelemetry("pixelLiftingMotor now moving");
                        } else {
                            pixelLiftingMotor.setPower(0);
                            addActTelemetry("pixelLiftingMotor no longer moving");
                        }
                    }
                }
            }

            if (carWashMotor != null) {
                if (!gamepad2.a && !gamepad2.b) {
                    carWashMotor.setPower(0);
                } else {
                    if (gamepad2.a) {
                        carWashMotor.setPower(carWashPower);
                        addActTelemetry("carWashMotor now moving forward");
                    } else if (gamepad2.b && !touchSensor.isPressed()) {
                        carWashMotor.setPower(-carWashPower);
                        addActTelemetry("carWashMotor now moving backward");
                    }
                }
            }

            if (trayTiltingServo != null) {
                boolean isLT = (gamepad2.left_trigger > 0.25);
                if (isLT && !wasLT) {
                    if (trayTiltingServo.getPosition() != 0) {
                        trayTiltingServo.setPosition(0);
                        addActTelemetry("Set trayTiltingServo to 0");
                    } else {
                        trayTiltingServo.setPosition(1);
                        addActTelemetry("Set trayTiltingServo to 0.35");
                    }
                }
                wasLT = isLT;
            }

            if (pixelBackServo != null) {
                if (gamepad2.y && !wasY) {
                    if (pixelBackServo.getPosition() > 0.5 - 0.05 && pixelBackServo.getPosition() < 0.5 + 0.05) {
                        pixelBackServo.setPosition(0);
                        addActTelemetry("Set pixelBackServo to 0");
                    } else {
                        pixelBackServo.setPosition(0.5);
                        addActTelemetry("Set pixelBackServo to 0.5");
                    }
                }
                wasY = gamepad2.y;
            }

            if (pixelFrontServo != null) {
                if (gamepad2.x && !wasX) {
                    if (pixelFrontServo.getPosition() > 0.30 - 0.05 && pixelFrontServo.getPosition() < 0.30 + 0.05) {
                        pixelFrontServo.setPosition(0.05);
                        addActTelemetry("Set pixelFrontServo to 0.05");
                    } else {
                        pixelFrontServo.setPosition(0.30);
                        addActTelemetry("Set pixelFrontServo to 0.30");
                    }
                }
                wasX = gamepad2.x;
                addActTelemetry("Did not move pixelFrontServo");
            } else {
                addActTelemetry("pixelFrontServo not connected");
            }

            if (droneServo != null) {
                if (gamepad2.left_bumper && gamepad2.right_bumper) {
                    droneServo.setPosition(1);
                }
            }
        }
        requestOpModeStop();
    }
    public void addActTelemetry(String message){
        telemetry.addData("Last Action",message); // Last Action: message
        telemetry.update();
    }
}