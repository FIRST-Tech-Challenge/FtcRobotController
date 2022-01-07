package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;

public class HardwareClass {

    // DcMotors
    public DcMotorEx frontRight;
    public DcMotorEx backRight;
    public DcMotorEx frontLeft;
    public DcMotorEx backLeft;
    public DcMotor carousel;
    public DcMotorEx armJoint;

    public DigitalChannel grabberTouch = null;
    public DigitalChannel limitFront = null;
    public DigitalChannel limitRear = null;

    /* servos */
    public CRServo grabberRight = null;
    public CRServo grabberLeft = null;


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
        armJoint = hwMap.get(DcMotorEx.class, "armJoint");

        //Servos
        grabberLeft = hwMap.get(CRServo.class, "left");
        grabberRight = hwMap.get(CRServo.class, "right");

        //Sensors
        limitFront = hwMap.get(DigitalChannel.class, "armLimitFront");
        limitFront.setMode(DigitalChannel.Mode.INPUT);
        limitRear = hwMap.get(DigitalChannel.class, "armLimitRear");
        limitRear.setMode(DigitalChannel.Mode.INPUT);

        grabberTouch = hwMap.get(DigitalChannel.class, "grabberTouch");
        grabberTouch.setMode(DigitalChannel.Mode.INPUT);


        //Motor direction
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        //Motor modes
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Zero power behavior (Makes sure robot does not move at all when motors at 0 power)
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }



}
