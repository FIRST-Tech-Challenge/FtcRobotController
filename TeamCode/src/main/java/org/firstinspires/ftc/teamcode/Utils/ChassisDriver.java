package org.firstinspires.ftc.teamcode.Utils;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class ChassisDriver {
    static DcMotorEx lf, rf, lb, rb;
    IMU imu;
    public static void initializeMotors(DcMotorEx lf, DcMotorEx rf, DcMotorEx lb, DcMotorEx rb) {
        ChassisDriver.lf = lf;
        ChassisDriver.rf = rf;
        ChassisDriver.lb = lb;
        ChassisDriver.rb = rb;

        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public static void resetWheelEncoders(DcMotorEx lf, DcMotorEx rf, DcMotorEx lb, DcMotorEx rb) {
        DcMotor.RunMode lfRM = lf.getMode();
        DcMotor.RunMode rfRM = rf.getMode();
        DcMotor.RunMode lbRM = lb.getMode();
        DcMotor.RunMode rbRM = rb.getMode();

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(lfRM);
        rf.setMode(rfRM);
        lb.setMode(lbRM);
        rb.setMode(rbRM);
    }
}