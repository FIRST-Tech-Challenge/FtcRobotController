package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class Hware {
    //Declare motors
    public DcMotor rightFront = null;
    public DcMotor leftFront = null;
    public DcMotor rightBack = null;
    public DcMotor leftBack = null;

    public DcMotor leftArm;
    public DcMotor rightArm;
    public DcMotor rightLift;
    public DcMotor leftLift;
    //Declare servos
    public Servo rightElbow = null;
    public Servo leftElbow = null;
    public CRServo intakeRight = null;
    public CRServo intakeLeft = null;
    public Servo leftWrist = null;
    public Servo rightWrist = null;
    public Servo planeServo = null;
    ColorSensor leftSensor = null;
    ColorSensor rightSensor = null;
    public DcMotor verticalLeft = null;
    public DcMotor verticalRight = null;
    public DcMotor horizontal = null;

    //Declare Additional variables
    public double ticks = 751.8;
    public double liftNewTarget;
    public double armNewTarget;

    HardwareMap hardwareMap = null;
    public ElapsedTime runtime = new ElapsedTime();

    public Hware(HardwareMap hwMap) {
        initialize(hwMap);
    }
    private void initialize(HardwareMap hwMap) {
        hardwareMap = hwMap;

        //initialize Chassis
        leftFront = hardwareMap.get(DcMotor.class, "LF");
        rightFront = hardwareMap.get(DcMotor.class, "RF");
        leftBack = hardwareMap.get(DcMotor.class, "LB");
        rightBack = hardwareMap.get(DcMotor.class, "RB");
        verticalRight = hardwareMap.get(DcMotor.class, "VR");
        verticalLeft =hardwareMap.get(DcMotor.class, "VL");
        horizontal = hardwareMap.get(DcMotor.class, "H");

        //Set motor directions
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.FORWARD);

        //initialize slide motors
        rightArm = hardwareMap.get(DcMotor.class, "rightArm");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        leftArm = hardwareMap.get(DcMotor.class, "leftArm");
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightArm.setDirection(DcMotor.Direction.REVERSE);
        rightLift.setDirection(DcMotor.Direction.REVERSE);
        leftArm.setDirection(DcMotor.Direction.FORWARD);
        leftLift.setDirection(DcMotor.Direction.FORWARD);

        //Set motor modes
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Initialize Servos
        rightElbow = hardwareMap.get(Servo.class, "rightElbow");
        leftElbow = hardwareMap.get(Servo.class, "leftElbow");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");
        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        leftWrist = hardwareMap.get(Servo.class, "leftWrist");
        rightWrist = hardwareMap.get(Servo.class, "rightWrist");
        planeServo = hardwareMap.get(Servo.class, "planeLauncher");
        leftSensor = hardwareMap.get(ColorSensor.class,"leftSensor");
        rightSensor = hardwareMap.get(ColorSensor.class, "rightSensor");

        intakeRight.setDirection(CRServo.Direction.REVERSE);
        intakeLeft.setDirection(CRServo.Direction.FORWARD);


        //Set Zero Power Behavior
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}