package org.firstinspires.ftc.team417_PowerPlay;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

abstract public class BaseTeleOp extends BaseOpMode {
    boolean grabberClosed = false;
    double armPower = 0.0;
    int armLevel = 0;
    int armEncoderPosition = 0;

    boolean armManual = false;

    ElapsedTime time;
    private static double BUMPER_TIMEOUT = 0.3; // in seconds

    private static double LOWER_ARM_MANUAL_POWER = -0.2;
    private static double LOWER_ARM_AUTO_POWER = -0.4;
    private static double RAISE_ARM_MANUAL_POWER = 1.0 / 100.0;
    private static double RAISE_ARM_AUTO_POWER = 1.0 / 800.0;

    private static double DRIVING_SPEED = 0.7;

    public void initializeTeleOp() {
        initializeHardware();
        time = new ElapsedTime();
        time.reset();

        grabberServo.setPosition(GRABBER_OPEN);

    }

    public void driveUsingControllers() {
        double x = gamepad1.left_stick_x * DRIVING_SPEED;
        double y = -gamepad1.left_stick_y * DRIVING_SPEED;
        double turning = gamepad1.right_stick_x * DRIVING_SPEED;
        y *= 1 - (0.5 * gamepad1.right_trigger);
        x *= 1 - (0.5 * gamepad1.right_trigger);
        turning *= 1 - (0.5 * gamepad1.right_trigger);
        mecanumDrive(x, y, turning);
    }

    public double stickCurve(double x, double a, double d) {
        x = Math.signum(x) * Math.max(0, Math.min(1, (Math.abs(x) - d) / (1 - d)));
        return a * x + (0.75-a) * Math.pow(x, 3);
    }

    public void driveStickCurve() {
        double x = stickCurve(gamepad1.left_stick_x, 0.1, 0.1);
        double y = stickCurve(-gamepad1.left_stick_y, 0.1, 0.1);
        double turning = stickCurve(gamepad1.right_stick_x, 0.1, 0.2);
        y *= 1 - (0.5 * gamepad1.right_trigger);
        x *= 1 - (0.5 * gamepad1.right_trigger);
        turning *= 1 - (0.5 * gamepad1.right_trigger);
        mecanumDrive(x, y, turning);
    }

    public void newButtonDrive() {
        // reset arm encoder position
        if (gamepad2.x && gamepad2.b) {
            motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        // manual control for arm
        // gets overridden when either bumper on gamepad2 is pressed
        if (gamepad2.dpad_down) {
            armEncoderPosition -= 5;
            armManual = true;
        } else if (gamepad2.dpad_up) {
            armEncoderPosition += 5;
            armManual = true;
        }
        Range.clip(armEncoderPosition, MIN_ARM_POSITION, MAX_ARM_POSITION);
        motorArm.setTargetPosition(armEncoderPosition);
        if (armManual) {
            if (armEncoderPosition >= motorArm.getCurrentPosition()) {
                motorArm.setPower((armEncoderPosition - motorArm.getCurrentPosition()) * RAISE_ARM_MANUAL_POWER);
            } else {
                // if need to lower
                if (motorArm.getCurrentPosition() - armEncoderPosition > ARM_ENCODER_TOLERANCE) {
                    motorArm.setPower(LOWER_ARM_MANUAL_POWER);
                } else {
                    motorArm.setPower(0);
                }
            }
        }
<<<<<<< HEAD

=======
        
>>>>>>> 1f7cccba60dd2f10dd4938b9d733432d221bc320
        boolean leftPress = gamepad2.left_bumper;
        boolean rightPress = gamepad2.right_bumper;
        boolean armAtStack = gamepad2.y;

        if (leftPress || rightPress || armAtStack) {
            armManual = false;
        }

        if (time.time() > BUMPER_TIMEOUT) {
            if (leftPress) {
                armLevel--;
                time.reset();
            } else if (rightPress) {
                armLevel++;
                time.reset();
            }
        }

        if (armLevel > 4) {
            armLevel = 4;
        } else if (armLevel < 0) {
            armLevel = 0;
        }

        if (!armManual) {
            switch (armLevel) {
                case 0: armEncoderPosition = MIN_ARM_POSITION;
                    break;
                case 1: armEncoderPosition = GROUND_JUNCT_ARM_POSITION;
                    break;
                case 2: armEncoderPosition = LOW_JUNCT_ARM_POSITION;
                    break;
                case 3: armEncoderPosition = MID_JUNCT_ARM_POSITION;
                    break;
                case 4: armEncoderPosition = HIGH_JUNCT_ARM_POSITION;
                    break;
            }

            if (armEncoderPosition >= motorArm.getCurrentPosition()) {
                motorArm.setPower((armEncoderPosition - motorArm.getCurrentPosition()) * RAISE_ARM_AUTO_POWER);
            } else {
                // if need to lower
                if (motorArm.getCurrentPosition() - armEncoderPosition > ARM_ENCODER_TOLERANCE) {
                    motorArm.setPower(LOWER_ARM_AUTO_POWER);
                } else {
                    motorArm.setPower(0);
                }
            }
        }
    }

    public void driveGrabber() {
        grabberClosed = grabberToggle.toggle(gamepad2.a);

        if (grabberClosed) {
            grabberServo.setPosition(GRABBER_CLOSED);
        } else {
            grabberServo.setPosition(GRABBER_OPEN);
        }
    }

    public void doTelemetry() {
        telemetry.addData("Arm power", armPower);
        telemetry.addData("Arm target", armEncoderPosition);
        telemetry.addData("Arm current position", motorArm.getCurrentPosition());
        telemetry.addData("Grabber position", grabberServo.getPosition());
        telemetry.addData("Grabber closed", grabberClosed);
        telemetry.addData("Arm level", armLevel);
        telemetry.update();
    }
}
