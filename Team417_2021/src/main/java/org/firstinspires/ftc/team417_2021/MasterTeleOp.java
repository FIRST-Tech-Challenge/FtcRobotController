package org.firstinspires.ftc.team417_2021;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team417_2021.Resources.Toggler;

abstract public class MasterTeleOp extends MasterOpMode {

    int shoulderPos = 0;
    int elbowPos = 0;
    double wristPos = WRIST_POS;
    Toggler grabberToggle = new Toggler();
    boolean grabberState = false;

    boolean movingShoulder = false;
    boolean movingElbow = false;
    int shoulderTarget = 0;
    int elbowTarget = 0;
    int shoulderError;
    int elbowError;
    double shoulderPower;
    double elbowPower;

    double inches = 0;

    public void driveRobotUsingController() {
        double drivePower = - gamepad1.right_stick_y;

        double rotationalPower = gamepad1.left_stick_x;

        drivePower *= 1 - (0.8 * gamepad1.right_trigger);
        rotationalPower *= 1 - (0.8 * gamepad1.right_trigger);

        if (gamepad1.a) {
            inches = robotInches();
        }

        drive(drivePower, rotationalPower);
    }

    // returns average value of all drive encoders, converted to inches
    public double robotInches() {
        return ((float) motorFL.getCurrentPosition() + motorBL.getCurrentPosition() +
                motorFR.getCurrentPosition() + motorBR.getCurrentPosition() ) / ( 4 * COUNTS_PER_INCH );
    }

    // joysticks change target position of arm joints
    public void encoderControlArm() {
        shoulderPos += gamepad2.left_stick_y * 6;
        elbowPos += gamepad2.right_stick_y * 6;
        telemetry.addData("Shoulder", shoulderPos);
        telemetry.addData("Elbow", elbowPos);
        telemetry.update();
        runMotorToPosition(shoulderMotor, shoulderPos, .1);
        runMotorToPosition(elbowMotor, elbowPos, .1);
    }

    public void controlMechanisms() {
        // joysticks change power of arm joints
        shoulderMotor.setPower(gamepad2.left_stick_y);
        elbowMotor.setPower(gamepad2.right_stick_y);

        // if wrist servo position needs to be changed
        if (gamepad2.right_bumper) {
            wristPos += 0.1;
            wristServo.setPosition(wristPos);
        } else if (gamepad2.left_bumper) {
            wristPos -= 0.1;
            wristServo.setPosition(wristPos);
        }

        // control grabber
        grabberState = grabberToggle.toggle(gamepad2.a);
        if (grabberState) {
            grabberServo.setPosition(GRABBER_OUT);
        } else {
            grabberServo.setPosition(GRABBER_IN);
        }

        // carousel
        if (gamepad2.right_trigger != 0) {
            carouselMotor.setPower(1);
        } else if (gamepad2.left_trigger != 0) {
            carouselMotor.setPower(-1);
        } else {
            carouselMotor.setPower(0);
        }

        telemetry.addData("Wrist position", wristServo.getPosition());
        telemetry.addData("wrist target", wristPos);
        telemetry.addData("grabber", grabberServo.getPosition());
        telemetry.update();
    }

    public void armToPosition() {
        // control using joysticks
        if (!movingShoulder && !movingElbow) {
            shoulderMotor.setPower(gamepad2.left_stick_y);
            elbowMotor.setPower(gamepad2.right_stick_y);
        }

        // setting target positions
        if (gamepad2.x) {
            movingShoulder = true;
            movingElbow = true;
            shoulderTarget = SHOULDER_LEVEL_1;
            elbowTarget = ELBOW_LEVEL_1;

        }
        else if (gamepad2.y) {
            movingShoulder = true;
            movingElbow = true;
            shoulderTarget = SHOULDER_LEVEL_3;
            elbowTarget = ELBOW_LEVEL_3;
        }

        // controlling shoulder
        if (Math.abs(shoulderTarget - shoulderMotor.getCurrentPosition()) > 5 && movingShoulder) {
            // finds error and inputs into PID filter to find power to set motor to
            shoulderError = shoulderTarget - shoulderMotor.getCurrentPosition();

            shoulderFilter.roll(shoulderError);

            shoulderPower = shoulderFilter.getFilteredValue();

            shoulderMotor.setPower(shoulderPower);

        }
        // if it's close to the target and moving, or if B is pressed, stop
        if ((Math.abs(shoulderTarget - shoulderMotor.getCurrentPosition()) <= 5 && movingShoulder)
                || gamepad2.b) {
            shoulderMotor.setPower(0.0);
            movingShoulder = false;
        }

        // controlling elbow
        if ((Math.abs(elbowTarget - elbowMotor.getCurrentPosition()) > 5 && movingElbow)
                || gamepad2.b) {
            elbowError = elbowTarget - elbowMotor.getCurrentPosition();

            elbowFilter.roll(elbowError);

            elbowPower = elbowFilter.getFilteredValue();

            elbowMotor.setPower(elbowPower);
        }
        if (Math.abs(elbowTarget - elbowMotor.getCurrentPosition()) >= 5 && movingElbow) {
            elbowMotor.setPower(0.0);
            movingElbow = false;
        }
    }
}
