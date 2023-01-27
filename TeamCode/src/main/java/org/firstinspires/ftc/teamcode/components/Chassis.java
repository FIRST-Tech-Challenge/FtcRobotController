package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Chassis {
    public DcMotor motorFL;
    public DcMotor motorFR;
    public DcMotor motorBL;
    public DcMotor motorBR;
    public IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    private ElapsedTime runtime = new ElapsedTime();

    Telemetry telemetry;
    public Chassis(DcMotor mFL, DcMotor mFR, DcMotor mBL, DcMotor mBR, IMU imu, Telemetry t){
        this.motorFL = mFL;
        this.motorFR = mFR;
        this.motorBL = mBL;
        this.motorBR = mBR;
        this.imu = imu;
        this.telemetry = t;
    }

    public void init() {
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

        //IMU
        IMU.Parameters parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
            )
        );

        imu.initialize(parameters);
        imu.resetYaw();
    }

    public double robotAngle() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public void joyStick(Gamepad gamepad) {
        double left_y = gamepad.left_stick_y;
        double left_x = gamepad.left_stick_x;
        double strafe_side = gamepad.right_stick_x;

        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;

        if (Math.abs(Math.atan2(Math.abs(left_y), Math.abs(left_x))) < Math.PI/14.0 && strafe_side == 0) {
            if (gamepad.right_bumper) {
                leftFrontPower = -1;
                rightFrontPower = 1;
                leftBackPower = 1;
                rightBackPower = -1;
            } else {
                leftFrontPower = -left_x * 0.8 - strafe_side * 0.6;
                rightFrontPower = left_x * 0.8 + strafe_side * 0.6;
                leftBackPower = left_x * 0.8 - strafe_side * 0.6;
                rightBackPower = -left_x * 0.8 + strafe_side * 0.6;
            }
        } else if (Math.abs(Math.atan2(Math.abs(left_x), Math.abs(left_y))) < Math.PI/14.0 && strafe_side == 0) {
             if (gamepad.right_bumper) {
                 leftFrontPower = 1;
                 rightFrontPower = 1;
                 leftBackPower = 1;
                 rightBackPower = 1;
             } else {
                 leftFrontPower = left_y * 0.8 - strafe_side * 0.6;
                 rightFrontPower = left_y * 0.8 + strafe_side * 0.6;
                 leftBackPower = left_y * 0.8 - strafe_side * 0.6;
                 rightBackPower = left_y * 0.8 + strafe_side * 0.6;
             }
        } else {
            leftFrontPower = (left_y - left_x) * 0.7 - strafe_side * 0.7;
            rightFrontPower = (left_y + left_x) * 0.7 + strafe_side * 0.7;
            leftBackPower = (left_y + left_x) * 0.7 - strafe_side * 0.7;
            rightBackPower = (left_y - left_x) * 0.7 + strafe_side * 0.7;
        }

        double max = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)), Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        if (gamepad.left_bumper) {
            motorFL.setPower(leftFrontPower * 0.35);
            motorFR.setPower(rightFrontPower * 0.35);
            motorBL.setPower(leftBackPower * 0.35);
            motorBR.setPower(rightBackPower * 0.35);
        } else {
            motorFL.setPower(leftFrontPower);
            motorFR.setPower(rightFrontPower);
            motorBL.setPower(leftBackPower);
            motorBR.setPower(rightBackPower);
        }
    }

    public void runToPosition(int FL, int FR, int BL, int BR) {
        runtime.reset();

        motorFL.setTargetPosition(FL);
        motorFR.setTargetPosition(FR);
        motorBL.setTargetPosition(BL);
        motorBR.setTargetPosition(BR);

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(motorFL.isBusy() || motorFR.isBusy() || motorBL.isBusy() || motorBR.isBusy()){
            if (Math.abs(motorFL.getCurrentPosition() - FL) < 300 || Math.abs(motorFR.getCurrentPosition() - FR) < 300 || Math.abs(motorBL.getCurrentPosition() - BL) < 300 || Math.abs(motorBR.getCurrentPosition() - BR) < 300) {
                motorFL.setPower(0.25);
                motorFR.setPower(0.25);
                motorBL.setPower(0.25);
                motorBR.setPower(0.25);
            } else {
                motorFL.setPower(0.6/(1+Math.pow(3, 3 * -runtime.seconds())));
                motorFR.setPower(0.6/(1+Math.pow(3, 3 * -runtime.seconds())));
                motorBL.setPower(0.6/(1+Math.pow(3, 3 * -runtime.seconds())));
                motorBR.setPower(0.6/(1+Math.pow(3, 3 * -runtime.seconds())));
            }
        }
    }

    public void runToAngle(double angle) {
        double difference = angle - robotAngle();
        while(Math.abs(difference) > 3){
            double power = (angle > robotAngle()) ? 0.3 : -0.3;
            turn(power);
            difference = robotAngle() - angle;

            telemetry.addData("yaw", robotAngle());
            telemetry.addData("difference", difference);
            telemetry.update();
        }
    }

    public void forward(double power) {
        motorFL.setPower(power);
        motorFR.setPower(power);
        motorBL.setPower(power);
        motorBR.setPower(power);
    }

    public void strafe(double power) {
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFL.setPower(power);
        motorFR.setPower(-power);
        motorBL.setPower(-power);
        motorBR.setPower(power);
    }

    public void turn(double power) {
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFL.setPower(power);
        motorFR.setPower(-power);
        motorBL.setPower(power);
        motorBR.setPower(-power);
    }

    public void stop() {
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }

    public void resetEncoder() {
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
