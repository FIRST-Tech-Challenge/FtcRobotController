package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareMap9010{

    //Constructor
    public HardwareMap9010(HardwareMap m){ hwMap = m; }

    public HardwareMap hwMap;

    //motors
    public DcMotor wheelFrontRight = null;
    public DcMotor wheelFrontLeft = null;
    public DcMotor wheelBackRight = null;
    public DcMotor wheelBackLeft = null;

    public DcMotor Slide = null;
    public DcMotor Turret = null;
    public DcMotor Vertical = null;
    //public DcMotor wobbleLift = null;

    //public Servo trigger = null;
    public Servo hopperdoor = null;
    public Servo Encoders = null;
    public Servo grabberclaw = null;
    // public Servo IntakeServo = null;
    public CRServo Spinner = null;
    public Servo WeedWhackerRight = null;
    //public Servo wobbleGrabber = null;
    //public Servo servoRingCounter = null;
    public ColorSensor sensorColor = null;
    public DistanceSensor sensorDistance = null;

    public void createHardware(){

        wheelFrontRight = hwMap.get(DcMotor.class, "rfWheel");
        wheelFrontLeft = hwMap.get(DcMotor.class, "lfWheel");
        wheelBackRight = hwMap.get(DcMotor.class, "rrWheel");
        wheelBackLeft = hwMap.get(DcMotor.class, "lrWheel");

        wheelFrontRight.setDirection(DcMotor.Direction.FORWARD);
        wheelBackRight.setDirection(DcMotor.Direction.REVERSE);
        wheelBackLeft.setDirection(DcMotor.Direction.FORWARD);
        wheelFrontLeft.setDirection(DcMotor.Direction.REVERSE);

        wheelFrontRight.setPower(0);
        wheelBackRight.setPower(0);
        wheelFrontLeft.setPower(0);
        wheelBackLeft.setPower(0);

        wheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        Slide = hwMap.get(DcMotor.class, "Slide");
        Turret = hwMap.get(DcMotor.class, "Turret");
        //intake = hwMap.get(DcMotor.class, "intake");
        Vertical = hwMap.get(DcMotor.class, "Vertical");
        //wobbleLift = hwMap.get(DcMotor.class, "WobbleLift");
        //wobbleLift.setPower(0);
        //wobbleLift.setTargetPosition(0);


        //shooter.setDirection(DcMotor.Direction.FORWARD);
        Slide.setDirection(DcMotor.Direction.FORWARD);
        Turret.setDirection(DcMotor.Direction.FORWARD);
        Vertical.setDirection(DcMotor.Direction.FORWARD);
        //shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //intake.setDirection(DcMotor.Direction.REVERSE);
        //wobbleLift.setDirection(DcMotor.Direction.FORWARD);
        //wobbleLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //spinner.setPower(0);

        //shooter.setPower(0);
        Slide.setPower(0);
        Turret.setPower(0);
        Vertical.setPower(0);
        //intake.setPower(0);

        sensorColor = hwMap.get(ColorSensor.class, "clawdistance");
        sensorDistance = hwMap.get(DistanceSensor.class, "clawdistance");
        //wobbleLift.setPower(0);

        Encoders = hwMap.get(Servo.class,"Encoders");
        Spinner = hwMap.get(CRServo.class,"Spinner");
        //wobbleGrabber = hwMap.get(Servo.class,"WobbleGrab");
        //WobbleLiftServo = hwMap.get(Servo.class,"WobbleLiftServo");
        grabberclaw = hwMap.get(Servo.class,"grabberclaw");
        //IntakeServo = hwMap.get(CRServo.class,"IntakeServo");
        Spinner.setPower(0);
        //WeedWhackerRight = hwMap.get(Servo.class,"WeedWhackerRight");
        //WeedWhackerLeft = hdwMap.get(CRServo.class,"WeedWhackerLeft");

        Encoders.setPosition(0.35);
        //trigger.setPosition(0.5);
        //servoRingCounter.setPosition(0.0);

    }
}