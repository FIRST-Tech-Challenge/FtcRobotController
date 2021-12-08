package org.firstinspires.ftc.team417_2021;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team417_2021.Resources.Toggler;

abstract public class MasterTeleOp extends MasterOpMode {

    int shoulderPos = 0;
    int elbowPos = 0;
    double wristPos = 0.0;
    Toggler grabberToggle = new Toggler();
    boolean grabberState = false;

    double inches = 0;

    public void driveRobotUsingController() {
        double drivePower = - gamepad1.right_stick_y;

        double rotationalPower = gamepad1.left_stick_x;

        drivePower *= 1 - (0.8 * gamepad1.right_trigger);
        rotationalPower *= 1 - (0.8 * gamepad1.right_trigger);

        /*telemetry.addData("Inches", robotInches() - inches);
        telemetry.addData("heading", robot.getCorrectedHeading());
        telemetry.update();*/
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
        if (gamepad2.dpad_up) {
            wristPos += 0.1;
            wristServo.setPosition(wristPos);
        } else if (gamepad2.dpad_down){
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
            carouselMotor.setPower(0.8);
        } else if (gamepad2.left_trigger != 0) {
            carouselMotor.setPower(-0.8);
        } else {
            carouselMotor.setPower(0);
        }
        telemetry.addData("Shoulder", shoulderMotor.getCurrentPosition());
        telemetry.addData("Elbow", elbowMotor.getCurrentPosition());
        telemetry.addData("Wrist", wristServo.getPosition());
        telemetry.addData("grabber", grabberServo.getPosition());
        telemetry.update();
/*
        telemetry.addData("toggle", grabberState);
        telemetry.addData("shoulder target", shoulderPos);
        telemetry.addData("elbow target", elbowPos);

        telemetry.addData("Shoulder", shoulderMotor.getCurrentPosition());
        telemetry.addData("Elbow", elbowMotor.getCurrentPosition());
        telemetry.addData("Wrist", wristServo.getPosition());
        telemetry.addData("shoulder", gamepad2.left_stick_y);
        telemetry.addData("elbow", gamepad2.right_stick_y);
        telemetry.update();*/

    }


}
