package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Device;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.internal.network.DeviceNameListener;

public class HardwareMap {

    public DcMotor leftfrontmotor = null;
    public DcMotor leftbackmotor = null;
    public DcMotor rightfrontmotor = null;
    public DcMotor rightbackmotor = null;

    HardwareMap hardwareMap = null;
    public ElapsedTime runtime = new ElapsedTime();

    public HardwareMap(HardwareMap hwMap) {
        initialize(hwMap);
    }

    private void initialize(HardwareMap hwMap){
        hardwareMap = hwMap;

        rightfrontmotor = hardwareMap.get(DcMotor.class, Devicename: "rightfrontmotor");

        rightfrontmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightbackmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftfrontmotor.setDirection(DcMotor.Direction.REVERSE);
        leftbackmotor.setDirection(DcMotor.Direction.REVERSE);

        leftbackmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftbackmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightfrontmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightbackmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftbackmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftbackmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfrontmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightbackmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightfrontmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftfrontmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftfrontmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftbackmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftfrontmotor.setPower(0);
        leftbackmotor.setPower(0);
        rightfrontmotor.setPower(0);
        rightbackmotor.setPower(0);

    }
}