package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.motors.GoBILDA5202Series;

public class Hardware {
    final public ElapsedTime timePassed = new ElapsedTime();
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public void init(HardwareMap hardwareMap) {
        try {
            frontLeft = hardwareMap.dcMotor.get("frontLeftMotor");
            frontLeft.setDirection(DcMotor.Direction.FORWARD);
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontLeft.setPower(0);
        } catch (Exception e) {
            telemetry.addData("FrontLeftMotor: ", "Error");
            telemetry.update();
        }
        try {
            frontRight = hardwareMap.dcMotor.get("frontRightMotor");
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setPower(0);
        } catch (Exception e) {
            telemetry.addData("FrontRightMotor: ", "Error");
            telemetry.update();
        }
        try {
            backRight = hardwareMap.dcMotor.get("backRightMotor");
            backRight.setDirection(DcMotor.Direction.FORWARD);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setPower(0);
        } catch (Exception e) {
            telemetry.addData("BackRightMotor: ", "Error");
            telemetry.update();
        }
        try {
            backLeft = hardwareMap.dcMotor.get("backLeftMotor");
            backLeft.setDirection(DcMotor.Direction.FORWARD);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setPower(0);
        } catch (Exception e) {
            telemetry.addData("BackLeftMotor: ", "Error");
            telemetry.update();
        }
    }

    public void drive(double power){
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }
    public void drive(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower){
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }
}