package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareClass {

    // DcMotors
    public DcMotorEx frontRight;
    public DcMotorEx backRight;
    public DcMotorEx frontLeft;
    public DcMotorEx backLeft;
    public DcMotor carousel;
    public DigitalChannel limitFront = null;
    public DigitalChannel limitRear = null;

    /* servos */
    public CRServo rightGrabber = null;
    public CRServo leftGrabber = null;


    HardwareMap hardwareMap = null;
    public ElapsedTime runTime = new ElapsedTime();

    public HardwareClass() {

    }

    public void init(HardwareMap hwMap) {

        hardwareMap = hwMap;

        //Motors
        frontLeft = hwMap.get(DcMotorEx.class, "leftFront");
        backLeft = hwMap.get(DcMotorEx.class, "leftRear");
        frontRight = hwMap.get(DcMotorEx.class, "rightFront");
        backRight = hwMap.get(DcMotorEx.class, "rightRear");
        carousel = hwMap.get(DcMotor.class, "carousel");

        limitFront = hwMap.get(DigitalChannel.class, "armLimitFront");
        limitRear = hwMap.get(DigitalChannel.class, "armLimitRear");

        //Motor direction
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }



}
