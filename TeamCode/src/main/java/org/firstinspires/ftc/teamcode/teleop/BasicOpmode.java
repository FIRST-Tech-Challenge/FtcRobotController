package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Basic Opmode", group = "Drive")
public class BasicOpmode extends LinearOpMode{
    DcMotorEx lf = null;
    DcMotorEx rf = null;
    DcMotorEx lb = null;
    DcMotorEx rb = null;
    @Override
    public void runOpMode(){
        lf = hardwareMap.get(DcMotorEx.class, "left_front"); // left front motor/wheel
        rf = hardwareMap.get(DcMotorEx.class, "right_front"); // right front motor/wheel
        lb = hardwareMap.get(DcMotorEx.class, "left_back"); // left back motor/wheel
        rb = hardwareMap.get(DcMotorEx.class, "right_back"); // right back motor/wheel
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setPower(1);
        rf.setPower(1);
        lb.setPower(1);
        rb.setPower(1);
    }
}
