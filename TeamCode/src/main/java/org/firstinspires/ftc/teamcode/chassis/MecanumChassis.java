package org.firstinspires.ftc.teamcode.chassis;

import static org.firstinspires.ftc.teamcode.Constants.driveStickThreshold;
import static org.firstinspires.ftc.teamcode.Constants.sensitivity;
import static org.firstinspires.ftc.teamcode.Constants.slowModeSensitivity;
import static org.firstinspires.ftc.teamcode.Constants.triggerThreshold;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MecanumChassis extends Chassis {
    ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    @Override
    public void init(HardwareMap hardwareMap) {
        super.init(hardwareMap);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void run(Gamepad gamepad){
        //This works, just trust me on it. Slack me or something if you need a full explanation.
        double flPower = (gamepad.left_stick_x - gamepad.left_stick_y + gamepad.right_stick_x)*driveStickThreshold;
        double frPower = (-gamepad.left_stick_x - gamepad.left_stick_y - gamepad.right_stick_x)*driveStickThreshold;
        double blPower = (-gamepad.left_stick_x - gamepad.left_stick_y + gamepad.right_stick_x)*driveStickThreshold;
        double brPower = (gamepad.left_stick_x - gamepad.left_stick_y - gamepad.right_stick_x)*driveStickThreshold;

        //This bit seems complicated, but it just gets the maximum absolute value of all the motors.
        double maxPower = Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)), Math.max(Math.abs(blPower), Math.abs(brPower)));

        //If maxPower is less than 1, make it 1. This allows for slower movements.
        maxPower = Math.max(maxPower, 1);

        //Make all of them proportional to the greatest value and factor in the sensitivity.
        flPower = (flPower / maxPower) * sensitivity;
        frPower = (frPower / maxPower) * sensitivity;
        blPower = (blPower / maxPower) * sensitivity;
        brPower = (brPower / maxPower) * sensitivity;

        //Actually set them

        if (gamepad.left_trigger > triggerThreshold ) {
            frontLeft.setPower(flPower);
            frontRight.setPower(frPower);
            backLeft.setPower(blPower);
            backRight.setPower(brPower);
        }
        else {
            frontLeft.setPower(flPower*slowModeSensitivity);
            frontRight.setPower(frPower*slowModeSensitivity);
            backLeft.setPower(blPower*slowModeSensitivity);
            backRight.setPower(brPower*slowModeSensitivity);
        }

    }

    // Movement functions
    public void delay(int time) {
        double startTime = elapsedTime.milliseconds();
        while (elapsedTime.milliseconds() - startTime < time) {
        }
    }

    public void moveForwardByTime(double power, int time) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
        delay(time);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void moveBackwardByTime(double power, int time) {
        frontLeft.setPower(-power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(-power);
        delay(time);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void strafeRightByTime(double power, int time){
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(power);
        delay(time);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void strafeLeftByTime(double power, int time){
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(-power);
        delay(time);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void turnRightByTime(double power, int time){
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(power);
        backRight.setPower(-power);
        delay(time);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void turnLeftByTime(double power, int time){
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(-power);
        backRight.setPower(power);
        delay(time);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void strafeRightWithEncoders(double power, int count){
        int start = frontLeft.getCurrentPosition();
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(power);
        while(frontLeft.getCurrentPosition() < start + count) {
            // Wait until movement is done
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void strafeLeftWithEncoders(double power, int count){
        int start = frontRight.getCurrentPosition();
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(-power);
        while(frontRight.getCurrentPosition() < start + count) {
            // Wait until movement is done
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void moveBackwardWithEncoders(double power, int count){
        int start = frontLeft.getCurrentPosition();
        frontLeft.setPower(-power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(-power);
        while(frontLeft.getCurrentPosition() > start - count) {
            // Wait until movement is done
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void moveForwardWithEncoders(double power, int count){
        int start = frontLeft.getCurrentPosition();
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
        while(frontLeft.getCurrentPosition() < start + count) {
            // Wait until movement is done
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void turnLeftWithEncoders(double power, int count){
        int start = frontRight.getCurrentPosition();
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(-power);
        backRight.setPower(power);
        while(frontRight.getCurrentPosition() < start + count) {
            // Wait until movement is done
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void turnRightWithEncoders(double power, int count){
        int start = frontLeft.getCurrentPosition();
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(power);
        backRight.setPower(-power);
        while(frontLeft.getCurrentPosition() < start + count) {
            // Wait until movement is done
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
