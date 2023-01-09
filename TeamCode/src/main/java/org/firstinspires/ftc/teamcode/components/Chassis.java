package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Chassis {
    public static DcMotor motorFL;
    public static DcMotor motorFR;
    public static DcMotor motorBL;
    public static DcMotor motorBR;

    public static final double speed = 0.6;

    public Chassis(){}

    public static void init(DcMotor mFL, DcMotor mFR, DcMotor mBL, DcMotor mBR) {
        Chassis.motorFL = mFL;
        Chassis.motorFR = mFR;
        Chassis.motorBL = mBL;
        Chassis.motorBR = mBR;

        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.REVERSE);

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static void joyStick(Gamepad gamepad) {
        double left_y = (Math.abs(gamepad.left_stick_y) < 0.2) ? 0 : gamepad.left_stick_y;
        double left_x = gamepad.left_stick_x;
        double strafe_side = gamepad.right_stick_x;

        double leftFrontPower = left_y - left_x - strafe_side;
        double rightFrontPower = left_y + left_x + strafe_side;
        double leftBackPower = left_y + left_x - strafe_side;
        double rightBackPower = left_y - left_x + strafe_side;

        double max = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)), Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        motorFL.setPower(leftFrontPower * speed);
        motorFR.setPower(rightFrontPower * speed);
        motorBL.setPower(leftBackPower * speed);
        motorBR.setPower(rightBackPower * speed);
    }

    public static void forward(double power) {
        motorFL.setPower(power * speed);
        motorFR.setPower(power * speed);
        motorBL.setPower(power * speed);
        motorBR.setPower(power * speed);
    }

    public static void strafe(double power) {
        motorFL.setPower(power * speed);
        motorFR.setPower(-power * speed);
        motorBL.setPower(-power * speed);
        motorBR.setPower(power * speed);
    }

    public static void turn(double power) {
        motorFL.setPower(power * speed);
        motorFR.setPower(-power * speed);
        motorBL.setPower(power * speed);
        motorBR.setPower(-power * speed);
    }

    public static void stop() {
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }

    public static void runToPosition(int FL, int FR, int BL, int BR) {
        motorFL.setTargetPosition(FL);
        motorFR.setTargetPosition(FR);
        motorBL.setTargetPosition(BL);
        motorBR.setTargetPosition(BR);

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(motorFL.isBusy() || motorFR.isBusy() || motorBL.isBusy() || motorBR.isBusy()){
            if (motorFL.isBusy()) motorFL.setPower(0.5 * speed);
            if (motorFR.isBusy()) motorFR.setPower(0.5 * speed);
            if (motorBL.isBusy()) motorBL.setPower(0.5 * speed);
            if (motorBR.isBusy()) motorBR.setPower(0.5 * speed);
        }
    }
}
