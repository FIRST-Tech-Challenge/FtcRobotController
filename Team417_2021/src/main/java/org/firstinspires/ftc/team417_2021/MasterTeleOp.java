package org.firstinspires.ftc.team417_2021;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team417_2021.Resources.Toggler;

abstract public class MasterTeleOp extends MasterOpMode {

    int shoulderPos = 0;
    int elbowPos = 0;
    double wristPos = 0.0;
    Toggler grabberToggle = new Toggler();
    boolean grabberState = false;
    boolean armRunning = false;
    final int SHOULDER_PICK_UP = -1340;
    final int ELBOW_PICK_UP = 1900;
    final int SHOULDER_LEVEL_1 = -1340;
    final int ELBOW_LEVEL_1 = 1900;

    public void driveRobotUsingController() {
        double drivePower = - gamepad1.right_stick_y;

        double rotationalPower = gamepad1.left_stick_x;

        drivePower *= 1 - (0.8 * gamepad1.right_trigger);
        rotationalPower *= 1 - (0.8 * gamepad1.right_trigger);
        /*telemetry.addData("FL", motorFL.getCurrentPosition());
        telemetry.addData("FR", motorFR.getCurrentPosition());
        telemetry.addData("BL", motorBL.getCurrentPosition());
        telemetry.addData("BR", motorBR.getCurrentPosition());
        telemetry.update();
*/
        drive(drivePower, rotationalPower);
    }

    public void simpleControlArm() {
        shoulderPos += gamepad2.left_stick_y * 6;
        elbowPos += gamepad2.right_stick_y * 6;
        telemetry.addData("Shoulder", shoulderPos);
        telemetry.addData("Elbow", elbowPos);
        telemetry.update();
        runMotorToPosition(shoulderMotor, shoulderPos, .1);
        runMotorToPosition(elbowMotor, elbowPos, .1);
    }

    public void controlArmPower() {
        shoulderMotor.setPower(gamepad2.left_stick_y);
        elbowMotor.setPower(gamepad2.right_stick_y);
        if (gamepad2.dpad_up) {
            wristPos += 0.1;
            wristServo.setPosition(wristPos);
        } else if (gamepad2.dpad_down){
            wristPos -= 0.1;
            wristServo.setPosition(wristPos);
        }
        grabberState = grabberToggle.toggle(gamepad2.a);
        if (grabberState) {
            grabberServo.setPosition(GRABBER_OUT);
        } else {
            grabberServo.setPosition(GRABBER_IN);
        }

        if (gamepad2.right_trigger != 0) {
            carouselMotor.setPower(0.5);
        } else if (gamepad2.left_trigger != 0) {
            carouselMotor.setPower(-0.5);
        } else {
            carouselMotor.setPower(0);
        }
/*
        if (gamepad2.x && !shoulderMotor.isBusy() && !elbowMotor.isBusy()) {
            shoulderPos = SHOULDER_PICK_UP;
            elbowPos = ELBOW_PICK_UP;

            shoulderMotor.setTargetPosition(SHOULDER_PICK_UP);
            shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulderMotor.setPower(0.5);

            elbowMotor.setTargetPosition(ELBOW_PICK_UP);
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbowMotor.setPower(0.5);

            armRunning = true;
        }*/
/*
        if (armRunning) {
            if (Math.abs(shoulderPos - shoulderMotor.getCurrentPosition()) > 10) {
                shoulderMotor.setPower(Math.signum(shoulderPos - shoulderMotor.getCurrentPosition()) * 0.7);
            }
            if (Math.abs(elbowPos - elbowMotor.getCurrentPosition()) > 10) {
                elbowMotor.setPower(Math.signum(elbowPos - elbowMotor.getCurrentPosition()) * 0.7);
            }
            if (!(Math.abs(elbowPos - elbowMotor.getCurrentPosition()) > 10)) {
                elbowMotor.setPower(0);
            }
            if (!(Math.abs(shoulderPos - shoulderMotor.getCurrentPosition()) > 10)) {
                shoulderMotor.setPower(0);
            }
            // if both have little error stop
            if (!(Math.abs(elbowPos - elbowMotor.getCurrentPosition()) > 10) &&
                    !(Math.abs(shoulderPos - shoulderMotor.getCurrentPosition()) > 10)) {
                armRunning = false;
            }
        }*/
        // if x and motor is not busy then run to position
/*
        if (armRunning) {
            if (Math.abs(shoulderPos - shoulderMotor.getCurrentPosition()) > 10) {
                shoulderMotor.setTargetPosition(SHOULDER_PICK_UP);
                shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shoulderMotor.setPower(0.5);
            }
            if (Math.abs(elbowPos - elbowMotor.getCurrentPosition()) > 10) {
                elbowMotor.setTargetPosition(ELBOW_PICK_UP);
                elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elbowMotor.setPower(0.5);            }
            if (!(Math.abs(elbowPos - elbowMotor.getCurrentPosition()) > 10)) {
                elbowMotor.setPower(0);
            }
            if (!(Math.abs(shoulderPos - shoulderMotor.getCurrentPosition()) > 10)) {
                shoulderMotor.setPower(0);
            }
            // if both have little error stop
            if (!(Math.abs(elbowPos - elbowMotor.getCurrentPosition()) > 10) &&
                    !(Math.abs(shoulderPos - shoulderMotor.getCurrentPosition()) > 10)) {
                armRunning = false;
            }
        }*/

        telemetry.addData("toggle", grabberState);
        telemetry.addData("shoulder target", shoulderPos);
        telemetry.addData("elbow target", elbowPos);

        telemetry.addData("Shoulder", shoulderMotor.getCurrentPosition());
        telemetry.addData("Elbow", elbowMotor.getCurrentPosition());
        telemetry.addData("Wrist", wristServo.getPosition());
        telemetry.addData("shoulder", gamepad2.left_stick_y);
        telemetry.addData("elbow", gamepad2.right_stick_y);
        telemetry.update();

    }

    public void runMotorToPosition(DcMotor motor, int targetPosition, double power) {

        motor.setTargetPosition(targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);


    }

}
