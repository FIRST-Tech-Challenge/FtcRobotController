package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.teamUtil.configNames;
import org.firstinspires.ftc.teamcode.teamUtil.robotConfig;

public class mecanumDriveBase {
    robotConfig r;

    private static DcMotorEx frontRight;
    private static DcMotorEx backRight;
    private static DcMotorEx backLeft;
    private static DcMotorEx frontLeft;

    public mecanumDriveBase(robotConfig r){
        this.r = r;

        frontRight = r.hardwareMap.get(DcMotorEx.class, configNames.frontRight);
        backRight = r.hardwareMap.get(DcMotorEx.class, configNames.backRight);
        backLeft = r.hardwareMap.get(DcMotorEx.class, configNames.backLeft);
        frontLeft = r.hardwareMap.get(DcMotorEx.class, configNames.frontLeft);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void rightStrafe(double Power){
        frontRight.setPower(-Power);
        backRight.setPower(Power);
        backLeft.setPower(-Power);
        frontLeft.setPower(Power);
    }
    public void leftStrafe(double Power){
        frontRight.setPower(Power);
        backRight.setPower(-Power);
        backLeft.setPower(Power);
        frontLeft.setPower(-Power);
    }
    public void drive(double leftPower, double rightPower){
        frontRight.setPower(rightPower);
        backRight.setPower(rightPower);
        backLeft.setPower(leftPower);
        frontLeft.setPower(leftPower);
    }

}