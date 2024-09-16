package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Hardware {
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backRight;
    public DcMotor backLeft;
    private IMU gyro;
    private final OpMode opMode;
    private static Hardware myInstance;

    private Hardware(OpMode opMode) {
        this.opMode = opMode;
    }

    public static Hardware getInstance(OpMode opMode){
        if(myInstance == null){
            myInstance = new Hardware(opMode);
        }
        return myInstance;
    }

    public void init(HardwareMap hardwareMap) {
        try {
            frontLeft = hardwareMap.dcMotor.get("fl");
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            opMode.telemetry.addData("FrontLeftMotor: ", "Initialized");
        } catch (Exception e) {
            opMode.telemetry.addData("FrontLeftMotor: ", "Error");
        } finally{
            opMode.telemetry.update();
        }

        try {
            frontRight = hardwareMap.dcMotor.get("fr");
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            opMode.telemetry.addData("FrontRightMotor: ", "Initialized.");
        } catch (Exception e) {
            opMode.telemetry.addData("FrontRightMotor: ", "Error");
        } finally {
            opMode.telemetry.update();
        }

        try {
            backRight = hardwareMap.dcMotor.get("br");
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            opMode.telemetry.addData("BackRightMotor: ", "Initialized.");
        } catch (Exception e) {
            opMode.telemetry.addData("BackRightMotor: ", "Error");
        } finally {
            opMode.telemetry.update();
        }

        try {
            backLeft = hardwareMap.dcMotor.get("bl");
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            opMode.telemetry.addData("BackLeftMotor: ", "Initialized.");
        } catch (Exception e) {
            opMode.telemetry.addData("BackLeftMotor: ", "Error");
        } finally {
            opMode.telemetry.update();
        }

        try {
            gyro = hardwareMap.get(IMU.class, "imu");
            // if vertically positioned
            gyro.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        } catch(Exception e) {
            opMode.telemetry.addData("Gyro ", "Error init");
        } finally {
            opMode.telemetry.update();
        }
    }

    /**
     * @return angle - left is ___, right is ___
     */
    public double getGyroAngle() {
        return gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void setMotorsToPower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    public void setTargets(int targetPos) {
        frontLeft.setTargetPosition(targetPos);
        frontRight.setTargetPosition(targetPos);
        backLeft.setTargetPosition(targetPos);
        backRight.setTargetPosition(targetPos);
    }

    public boolean notInRange(int targetPos) {
        return (notInRange(frontLeft, targetPos, 10) && notInRange(frontRight, targetPos, 10)
                && notInRange(backLeft, targetPos, 10) && notInRange(backRight, targetPos, 10));
    }

    public static boolean notInRange(DcMotor motor, int targetPos, int threshold) {
        return !(((Math.abs(motor.getCurrentPosition()) - threshold) <= Math.abs(targetPos)) &&
                ((Math.abs(motor.getCurrentPosition() + threshold)) >= Math.abs(targetPos)));
    }

    public void telemetryHardware() {
        opMode.telemetry.addData("FrontLeftPower: ", frontLeft.getPower());
        opMode.telemetry.addData("FrontRightPower: ", frontRight.getPower());
        opMode.telemetry.addData("backRightPower: ", backRight.getPower());
        opMode.telemetry.addData("backLeftPower: ", backLeft.getPower());

        opMode.telemetry.update();
    }

    public void setMotorsToRunToPosition() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
