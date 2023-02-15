package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.teamUtil.ConfigNames;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;

public class MecanumDriveBase {
    RobotConfig r;

    private static DcMotorEx frontRight;
    private static DcMotorEx backRight;
    private static DcMotorEx backLeft;
    private static DcMotorEx frontLeft;

    public MecanumDriveBase(RobotConfig r){
        this.r = r;

        frontRight = r.hardwareMap.get(DcMotorEx.class, ConfigNames.frontRight);
        backRight = r.hardwareMap.get(DcMotorEx.class, ConfigNames.backRight);
        backLeft = r.hardwareMap.get(DcMotorEx.class, ConfigNames.backLeft);
        frontLeft = r.hardwareMap.get(DcMotorEx.class, ConfigNames.frontLeft);
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