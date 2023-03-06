package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.Subsystem;
import org.firstinspires.ftc.teamcode.teamUtil.ConfigNames;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;

public class MecanumDriveBase extends Subsystem {
    RobotConfig r;

    private DcMotor frontRight;
    private DcMotorEx backRight;
    private DcMotorEx backLeft;
    private DcMotorEx frontLeft;

    private boolean fieldCentric = true;

    public MecanumDriveBase(RobotConfig r){
        this.r = r;
    }
    public MecanumDriveBase(){
        r = RobotConfig.getInstance();
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

    public void trueHolonomicDrive(double planarX, double planarY, double headingControl){
        if(fieldCentric) { //TODO handle imu/angle stuff here
            frontRight.setPower(planarX - planarY + headingControl);
            backRight.setPower(planarX + planarY - headingControl);
            backLeft.setPower(planarX - planarY - headingControl);
            frontLeft.setPower(planarX + planarY + headingControl);
        }
        else{
            frontRight.setPower(planarX - planarY + headingControl);
            backRight.setPower(planarX + planarY - headingControl);
            backLeft.setPower(planarX - planarY - headingControl);
            frontLeft.setPower(planarX + planarY + headingControl);
        }
    }

    public void setFieldCentric(boolean fieldCentric){
        this.fieldCentric = fieldCentric;
    }

    @Override
    public void init() {
        frontRight = r.opMode.hardwareMap.get(DcMotorEx.class, ConfigNames.frontRight);
        backRight = r.opMode.hardwareMap.get(DcMotorEx.class, ConfigNames.backRight);
        backLeft = r.opMode.hardwareMap.get(DcMotorEx.class, ConfigNames.backLeft);
        frontLeft = r.opMode.hardwareMap.get(DcMotorEx.class, ConfigNames.frontLeft);

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void read() {

    }

    @Override
    public void update() {

    }

    @Override
    public void close() {

    }
}