package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Main", group = "CenterStage")
public class TeleOp_Main extends Base {

    boolean wasX = false;
    boolean wasLT = false;
    double axial = 0.0;
    double lateral = 0.0;
    double yaw = 0.0;
    boolean TS = false;
    boolean wasTS = false;
    boolean isLT = false;
    double leftFrontPower = 0.0;
    double rightFrontPower = 0.0;
    double leftBackPower = 0.0;
    double rightBackPower = 0.0;
    double max = 0.0;
    static final double SPEED_MULTIPLIER = 0.75;
    static final double BASE_TURN_SPEED = 2.5;
    double slowdownMultiplier = 0.0;
    static final double CAR_WASH_POWER = 1.0;

    @Override
    public void runOpMode() {
        setup();
        if (trayTiltingServo != null) { trayTiltingServo.setPosition(0); }
        if (pixelLockingServo != null) { pixelLockingServo.setPosition(1); }

        while (opModeIsActive()) {
            // Slows down movement for better handling the more the right trigger is held down
            slowdownMultiplier = (1.0 - gamepad1.right_trigger);
//            if (gamepad1.left_stick_button || gamepad1.right_stick_button) { slowdownMultiplier *= 0.5; }

            axial = ((-gamepad1.left_stick_y * SPEED_MULTIPLIER) * slowdownMultiplier);
            lateral = ((gamepad1.left_stick_x * SPEED_MULTIPLIER) * slowdownMultiplier);
            yaw = ((gamepad1.right_stick_x * BASE_TURN_SPEED) * slowdownMultiplier);

            leftFrontPower = axial + lateral + yaw;
            rightFrontPower = axial - lateral - yaw;
            leftBackPower = axial - lateral + yaw;
            rightBackPower = axial + lateral - yaw;

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
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
            } else {
                print("WARNING:", "At least one drivetrain motor disconnected");
            }

            // Logic to raise or lower the lift
            if (pixelLiftingMotor != null) {
                addTelemetry("Current position: " + pixelLiftingMotor.getCurrentPosition());
                if (!gamepad2.dpad_up && !gamepad2.dpad_down || gamepad2.dpad_down && gamepad2.dpad_up) {
                    pixelLiftingMotor.setPower(0);
                } else {
                    if (touchSensor != null) {
                        TS = touchSensor.isPressed();
                    } else {
                        TS = false;
                        addTelemetry("Touch sensor not connected");
                    }
                    if (gamepad2.dpad_up && !gamepad2.dpad_down) {
                        if (pixelLiftingMotor.getCurrentPosition() < 5500) {
                            pixelLiftingMotor.setPower(1);
                            addTelemetry("pixelLiftingMotor now moving");
                        } else {
                            pixelLiftingMotor.setPower(0);
                            addTelemetry("pixelLiftingMotor no longer moving");
                        }
                    } else if (gamepad2.dpad_down && !gamepad2.dpad_up && !TS) {
                        if (pixelLiftingMotor.getCurrentPosition() > 0) {
                            pixelLiftingMotor.setPower(-1);
                            addTelemetry("pixelLiftingMotor now moving");
                        } else {
                            pixelLiftingMotor.setPower(0);
                            addTelemetry("pixelLiftingMotor no longer moving");
                        }
                    }
                }
            }

            // Logic to start and stop car wash
            if (carWashMotor != null) {
                if (!gamepad2.a && !gamepad2.b) {
                    carWashMotor.setPower(0);
                } else {
                    if (gamepad2.b) {
                        carWashMotor.setPower(CAR_WASH_POWER);
                        addTelemetry("carWashMotor now moving forward");
                    } else if (gamepad2.a) {
                        carWashMotor.setPower(-CAR_WASH_POWER);
                        addTelemetry("carWashMotor now moving backward");
                    }
                }
            }

            // Logic to tilt the pixel box back and forth
            if (trayTiltingServo != null) {
                isLT = (gamepad2.left_trigger > 0.25);
                if (isLT && !wasLT) {
                    if (trayTiltingServo.getPosition() != 0) {
                        trayTiltingServo.setPosition(0);
                        addTelemetry("Set trayTiltingServo to 0");
                    } else {
                        trayTiltingServo.setPosition(0.5);
                        addTelemetry("Set trayTiltingServo to 0.5");
                    }
                }
                wasLT = isLT;
            }

            // Logic to lock pixel(s) in the box
            if (pixelLockingServo != null) {
                if (gamepad2.x) {
                    pixelLockingServo.setPosition(1);
                } else { pixelLockingServo.setPosition(0); }
                wasX = gamepad2.x;
            }

            // Logic to launch drone
            if (droneServo != null) {
                if (gamepad2.left_bumper && gamepad2.right_bumper) {
                    droneServo.setPosition(0);
                    addTelemetry("Set droneServo to 0");
                }
            }
        }

        if (touchSensor != null) {
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
        }
        updateAll();
    }

    /**
     * Adds telemetry data from the last action
     * @param message Message to be sent
     */
    public void addTelemetry(String message){
        telemetry.addData("Last Action", message);
    }

    /**
     * Adds information messages to telemetry and updates it
     */
    public void updateAll() {
        telemetry.addData("Pixel Lifting Motor Position", pixelLiftingMotor.getCurrentPosition());
        if(trayTiltingServo == null) {
            telemetry.addData("Tray Tilting Servo", "Disconnected");
        }
        if (pixelLockingServo == null) {
            telemetry.addData("Pixel Front Servo", "Disconnected");
        }
        if (touchSensor == null) {
            telemetry.addData("Touch Sensor", "Disconnected");
        }
        telemetry.update();

    }
}
