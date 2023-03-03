package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

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

    public void MDrive(double vertical, double horizontal, double pivot, double heading){
        this.vertical = vertical;
        this.horizontal = horizontal;
        this.pivot = pivot;
        this.heading = heading;

        theta = 2 * Math.PI + Math.atan2(vertical,horizontal) - heading;
        power = Math.hypot(horizontal,vertical);

        sin = Math.sin(theta -Math.PI/4);
        cos = Math.cos(theta - Math.PI/4);
        max = Math.max(Math.abs(sin),Math.abs(cos));

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
