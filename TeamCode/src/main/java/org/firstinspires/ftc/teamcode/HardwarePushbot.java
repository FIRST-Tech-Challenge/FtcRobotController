package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class HardwarePushbot
{

    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;
    public DcMotor carousel;

    HardwareMap hwMap = null;

    public HardwarePushbot () {

    }

    public void init (HardwareMap ahwMap) {

        hwMap = ahwMap;

        //Motors
        frontLeft = hwMap.get(DcMotor.class, "leftFront");
        backLeft = hwMap.get(DcMotor.class, "leftRear");
        frontRight = hwMap.get(DcMotor.class, "rightFront");
        backRight = hwMap.get(DcMotor.class, "rightRear");
        carousel = hwMap.get(DcMotor.class, "carousel");

        //Motor direction
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        //Encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

}