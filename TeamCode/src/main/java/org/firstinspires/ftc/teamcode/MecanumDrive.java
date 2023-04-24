package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.lang.Math;

public class MecanumDrive{
    double max;
    double sin;
    double cos;
    double theta;
    double power;
    double vertical;
    double horizontal;
    double pivot;
    double heading;
    double FLPower;
    double FRPower;
    double BLPower;
    double BRPower;
    Project1Hardware robot;

    public MecanumDrive(Project1Hardware robot){
        this.robot = robot;
    }

    public void remote(double vertical, double horizontal, double pivot, double heading){
        this.vertical = vertical;
        this.horizontal = horizontal;
        this.pivot = pivot;
        this.heading = heading ;

        theta = 2 * Math.PI + Math.atan2(vertical,horizontal) - heading;
        power = Math.hypot(horizontal,vertical);

        sin = Math.sin(theta -Math.PI/4);
        cos = Math.cos(theta - Math.PI/4);
        max = Math.max(Math.abs(sin),Math.abs(cos));

        FLPower = power * (cos/max) + pivot;
        FRPower = power * sin/max - pivot;
        BLPower = power * -(sin/max) - pivot;
        BRPower = power * -(cos/max) + pivot;

        robot.frontLeft.setPower(-FLPower);
        robot.frontRight.setPower(-FRPower);
        robot.backLeft.setPower(BLPower);
        robot.backRight.setPower(BRPower);

        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //also mecanum drive but more organized - not tested yet
    public void remote2(Project1Hardware robot, double vertical, double horizontal, double pivot, double heading){
        robot.frontLeft.setDirection(DcMotor.Direction.FORWARD);
        robot.frontRight.setDirection(DcMotor.Direction.REVERSE);
        robot.backLeft.setDirection(DcMotor.Direction.FORWARD);
        robot.backRight.setDirection(DcMotor.Direction.REVERSE);

        this.vertical = vertical;
        this.horizontal = horizontal;
        this.pivot = pivot;
        this.heading = heading+(Math.PI/2);

        theta = 2 * Math.PI + Math.atan2(vertical,horizontal) - heading;
        power = Math.hypot(horizontal,vertical);

        sin = Math.sin(theta -Math.PI/4);
        cos = Math.cos(theta - Math.PI/4);
        max = Math.max(Math.abs(sin),Math.abs(cos));

        /*
        FLPower = power * (cos/max) + pivot;
        FRPower = power * (sin/max) - pivot;
        BLPower = power * (sin/max) + pivot;
        BRPower = power * (cos/max) - pivot;
        */
        FLPower = power * (cos/max) - pivot;
        FRPower = power * (sin/max) + pivot;
        BLPower = power * (sin/max) - pivot;
        BRPower = power * (cos/max) + pivot;

        robot.frontLeft.setPower(FLPower);
        robot.frontRight.setPower(FRPower);
        robot.backLeft.setPower(BLPower);
        robot.backRight.setPower(BRPower);

        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void part1(double theta, double pivot, double power){
        theta = 2 * Math.PI + (theta / 360 * 2 * Math.PI) - Math.PI / 2;

        sin = Math.sin(theta -Math.PI/4);
        cos = Math.cos(theta - Math.PI/4);
        max = Math.max(Math.abs(sin),Math.abs(cos));

        FLPower = power * (cos/max) + pivot;
        FRPower = power * sin/max - pivot;
        BLPower = power * -(sin/max) - pivot;
        BRPower = power * -(cos/max) + pivot;

        robot.frontLeft.setPower(-FLPower);
        robot.frontRight.setPower(-FRPower);
        robot.backLeft.setPower(BLPower);
        robot.backRight.setPower(BRPower);

        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive (double target, double power, double pivot, double distance){

        this.theta = Math.PI + (target * Math.PI/180);

        sin = Math.sin(theta -Math.PI/4);
        cos = Math.cos(theta - Math.PI/4);
        max = Math.max(Math.abs(sin),Math.abs(cos));

        int FL = robot.frontLeft.getCurrentPosition();
        int FR = robot.frontRight.getCurrentPosition();
        int BL = robot.backLeft.getCurrentPosition();
        int BR = robot.backRight.getCurrentPosition();

        double orig = FL;
        double cur = orig;

        while (Math.abs(cur-orig) <= distance){
            FL = robot.frontLeft.getCurrentPosition();
            FR = robot.frontRight.getCurrentPosition();
            BL = robot.backLeft.getCurrentPosition();
            BR = robot.backRight.getCurrentPosition();

            cur = FL;

            FLPower = power * -(cos/max) + pivot;
            FRPower = power * sin/max + pivot;
            BLPower = power * -(sin/max) + pivot;
            BRPower = power * cos/max + pivot;

            robot.frontLeft.setPower(-FLPower);
            robot.frontRight.setPower(-FRPower);
            robot.backLeft.setPower(BLPower);
            robot.backRight.setPower(BRPower);

            robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }
    }
}




/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.lang.Math;

public class MecanumDrive{
    double max;
    double sin;
    double cos;
    double theta;
    double power;
    double vertical;
    double horizontal;
    double pivot;
    double heading;
    double FLPower;
    double FRPower;
    double BLPower;
    double BRPower;
    Project1Hardware robot;

    public MecanumDrive(Project1Hardware robot){
        this.robot = robot;
    }

    public void remote(double vertical, double horizontal, double pivot, double heading){
        this.vertical = vertical;
        this.horizontal = horizontal;
        this.pivot = pivot;
        this.heading = heading ;

        theta = 2 * Math.PI + Math.atan2(vertical,horizontal) - heading;
        power = Math.hypot(horizontal,vertical);

        sin = Math.sin(theta -Math.PI/4);
        cos = Math.cos(theta - Math.PI/4);
        max = Math.max(Math.abs(sin),Math.abs(cos));

        FLPower = power * (cos/max) + pivot;
        FRPower = power * sin/max - pivot;
        BLPower = power * -(sin/max) - pivot;
        BRPower = power * -(cos/max) + pivot;

        robot.frontLeft.setPower(-FLPower);
        robot.frontRight.setPower(-FRPower);
        robot.backLeft.setPower(BLPower);
        robot.backRight.setPower(BRPower);

        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //also mecanum drive but more organized - not tested yet
    public void remote2(Project1Hardware robot, double vertical, double horizontal, double pivot, double heading){
        robot.frontLeft.setDirection(DcMotor.Direction.FORWARD);
        robot.frontRight.setDirection(DcMotor.Direction.REVERSE);
        robot.backLeft.setDirection(DcMotor.Direction.FORWARD);
        robot.backRight.setDirection(DcMotor.Direction.REVERSE);

        this.vertical = vertical;
        this.horizontal = horizontal;
        this.pivot = pivot;
        this.heading = heading+(Math.PI/2);

        theta = 2 * Math.PI + Math.atan2(vertical,horizontal) - heading;
        power = Math.hypot(horizontal,vertical);

        sin = Math.sin(theta -Math.PI/4);
        cos = Math.cos(theta - Math.PI/4);
        max = Math.max(Math.abs(sin),Math.abs(cos));

        /*
        FLPower = power * (cos/max) + pivot;
        FRPower = power * (sin/max) - pivot;
        BLPower = power * (sin/max) + pivot;
        BRPower = power * (cos/max) - pivot;
        *//*
        FLPower = power * (cos/max) - pivot;
                FRPower = power * (sin/max) + pivot;
                BLPower = power * (sin/max) - pivot;
                BRPower = power * (cos/max) + pivot;

                robot.frontLeft.setPower(FLPower);
                robot.frontRight.setPower(FRPower);
                robot.backLeft.setPower(BLPower);
                robot.backRight.setPower(BRPower);

                robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }

public void drive (double target, double power, double pivot, double distance){

        this.theta = Math.PI + (target * Math.PI/180);

        sin = Math.sin(theta -Math.PI/4);
        cos = Math.cos(theta - Math.PI/4);
        max = Math.max(Math.abs(sin),Math.abs(cos));

        int FL = robot.frontLeft.getCurrentPosition();
        int FR = robot.frontRight.getCurrentPosition();
        int BL = robot.backLeft.getCurrentPosition();
        int BR = robot.backRight.getCurrentPosition();

        double orig = FL;
        double cur = orig;

        while (Math.abs(cur-orig) <= distance){
        FL = robot.frontLeft.getCurrentPosition();
        FR = robot.frontRight.getCurrentPosition();
        BL = robot.backLeft.getCurrentPosition();
        BR = robot.backRight.getCurrentPosition();

        cur = FL;

        FLPower = power * -(cos/max) + pivot;
        FRPower = power * sin/max + pivot;
        BLPower = power * -(sin/max) + pivot;
        BRPower = power * cos/max + pivot;

        robot.frontLeft.setPower(-FLPower);
        robot.frontRight.setPower(-FRPower);
        robot.backLeft.setPower(BLPower);
        robot.backRight.setPower(BRPower);

        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }
        }
        }
 */