package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class HorizontalExtendo {
    DcMotorEx leftMotor, rightMotor;

    private final double multipleConstant = 1000;
    private final double motorSpeed = 0.7;


    public HorizontalExtendo(HardwareMap hardwareMap, String leftMotorName, String rightMotorName)
    {
        leftMotor = hardwareMap.get(DcMotorEx.class, leftMotorName);
        rightMotor = hardwareMap.get(DcMotorEx.class, rightMotorName);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

//    public HorizontalExtendo(String leftMotorName, String rightMotorName)
//    {
//        leftMotor = hardwareMap.get(DcMotorEx.class, leftMotorName);
//        rightMotor = hardwareMap.get(DcMotorEx.class, rightMotorName);
//
//        leftMotor.setTargetPosition(leftMotor.getCurrentPosition());
//        rightMotor.setTargetPosition(leftMotor.getCurrentPosition());
//
//        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        leftMotor.setPower(motorSpeed);
//        rightMotor.setPower(motorSpeed);
//    }

    public void move(double val)
    {
        leftMotor.setPower(val);
        rightMotor.setPower(val);
    }

    public void movePos(double val)
    {
        val *= multipleConstant;

        int pos = leftMotor.getCurrentPosition() + (int) val;

        leftMotor.setTargetPosition(pos);
        rightMotor.setTargetPosition(pos);


    }

    public int getPos()
    {
        return leftMotor.getCurrentPosition();
    }

    public void resetPos()
    {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }
}
