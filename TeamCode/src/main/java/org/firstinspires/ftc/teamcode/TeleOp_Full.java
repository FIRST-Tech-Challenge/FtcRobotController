package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Full", group = "CenterStage")
public class TeleOp_Full extends CSBase {

    // Declare OpMode members for each of the 4 motors.
    boolean wasY = false;
    boolean wasX = false;
    boolean wasLT = false;
    double axial = 0.0;
    double lateral = 0.0;
    double yaw = 0.0;
    boolean TS = false;
    boolean wasTS = false;
    final double speedMultiplier = 0.75;
    final double baseTurnSpeed = 2.5;
    double slowdownMultiplier = 0.0;


    @Override
    public void runOpMode() {
        setup(color.none, false);
        double carWashPower = 1.0;
        double[] backBounds = {0.3, 0.6};
        if (trayTiltingServo != null) {
            trayTiltingServo.setPosition(1);
        }

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
            if (lf != null) {
                lf.setPower(leftFrontPower);
                rf.setPower(rightFrontPower);
                lb.setPower(leftBackPower);
                rb.setPower(rightBackPower);
            }

            if (pixelLiftingMotor != null) {
                if (!gamepad2.dpad_up && !gamepad2.dpad_down || gamepad2.dpad_down && gamepad2.dpad_up) {
                    pixelLiftingMotor.setPower(0);
                } else {
                    if (touchSensor != null) {
                        TS = touchSensor.isPressed();
                    } else {
                        TS = false;
                        addActTelemetry("Touch sensor not connected");
                    }
                    if (gamepad2.dpad_up && !gamepad2.dpad_down) {
                        if (pixelLiftingMotor.getCurrentPosition() < 2450) {
                            pixelLiftingMotor.setPower(0.75);
                            addActTelemetry("pixelLiftingMotor now moving");
                        } else {
                            pixelLiftingMotor.setPower(0);
                            addActTelemetry("pixelLiftingMotor no longer moving");
                        }
                    }
                    if (gamepad2.dpad_down && !gamepad2.dpad_up && !TS) {
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
                    } else if (gamepad2.b) {
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
                        addActTelemetry("Set trayTiltingServo to 1");
                    }
                }
                wasLT = isLT;
            }

            if (pixelBackServo != null) {
                if (gamepad2.y && !wasY) {
                    if (pixelBackServo.getPosition() > backBounds[1] - 0.05 && pixelBackServo.getPosition() < backBounds[1] + 0.05) {
                        pixelBackServo.setPosition(backBounds[0]);
                        addActTelemetry("Set pixelBackServo to 0");
                    } else {
                        pixelBackServo.setPosition(backBounds[1]);
                        addActTelemetry("Set pixelBackServo to 0.6");
                    }
                }
                wasY = gamepad2.y;
            }

            if (pixelFrontServo != null) {
                if (gamepad2.x && !wasX) {
                    if (pixelFrontServo.getPosition() > 0.83 - 0.05 && pixelFrontServo.getPosition() < 0.83 + 0.05) {
                        pixelFrontServo.setPosition(0.5);
                        addActTelemetry("Set pixelFrontServo to 0.5");
                    } else {
                        pixelFrontServo.setPosition(0.83);
                        addActTelemetry("Set pixelFrontServo to 0.83");
                    }
                }
                wasX = gamepad2.x;
            }

            if (droneServo != null) {
                if (gamepad2.left_bumper && gamepad2.right_bumper) {
                    droneServo.setPosition(0);
                    addActTelemetry("Set droneServo to 0");
                }
            }
        }

        /*if (touchSensor != null) {
            if (!wasTS) {
                if (touchSensor.isPressed()) {
                    pixelLiftingMotor.setPower(0);
                    pixelLiftingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    wasTS = true;
                } else {
                    wasTS = false;
                }
            } else if (!touchSensor.isPressed()) {
                wasTS = false;
            }
        }*/
        requestOpModeStop();
    }
    public void addActTelemetry(String message){
        telemetry.addData("Last Action",message);
        telemetry.addData("Pixel Lifting Motor Position", pixelLiftingMotor.getCurrentPosition());
        if(trayTiltingServo == null) {
            telemetry.addData("Tray Tilting Servo", "Disconnected");
        }
        if (pixelFrontServo == null) {
            telemetry.addData("Pixel Front Servo", "Disconnected");
        }
        if (touchSensor == null) {
            telemetry.addData("Touch Sensor", "Disconnected");
        }
        telemetry.update();
    }
}