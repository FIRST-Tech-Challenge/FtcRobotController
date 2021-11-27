package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class zanehardware {

    public DcMotor Front_Left;
    public DcMotor Front_Right;
    public DcMotor Back_Left;
    public DcMotor Back_Right;
    public DcMotor Spinner;
    public DistanceSensor LeftDistance;
    public DistanceSensor RightDistance;
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public void init(HardwareMap ahwmap){
        hwMap = ahwmap;

        Front_Left = hwMap.get(DcMotor.class,"Front_Left");
        Front_Right = hwMap.get(DcMotor.class,"Front_Right");
        Back_Left = hwMap.get(DcMotor.class,"Back_Left");
        Back_Right = hwMap.get(DcMotor.class,"Back_Right");
        Spinner = hwMap.get(DcMotor.class,"Spinner");
        LeftDistance = hwMap.get(DistanceSensor.class,"fDistance");

        //reverse motors
        Front_Right.setDirection(DcMotorSimple.Direction.REVERSE);
        Front_Left.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void allMotorPower(double power){
        Front_Left.setPower(power);
        Front_Right.setPower(power);
        Back_Left.setPower(power);
        Back_Right.setPower(power);

    }
    public void leftMotorPower(double power){
        Front_Left.setPower(power);
        Back_Left.setPower(power);

    }
    public void rightMotorPower(double power){
        Front_Right.setPower(power);
        Back_Right.setPower(power);
    }

}
