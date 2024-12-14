package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware   {

    private static Hardware instance = null;
    private boolean enabled;

    private HardwareMap hardwareMap;

    // Chassis (Drivetrain, Pinpoint, Hubs)
    public DcMotorEx LF;
    public DcMotorEx RF;
    public DcMotorEx RB;
    public DcMotorEx LB;

    public GoBildaPinpointDriver pinPoint;

    // Deposit
    public DcMotorEx depositSlideRight;
    public DcMotorEx depositSlideLeft;

    public CRServo armRight;
    public CRServo armLeft;
    public CRServo claw;

    public AnalogInput armRightEnc;
    public AnalogInput armLeftEnc;
    public AnalogInput clawEnc;

    // Intake
    public DcMotorEx intakeSlide;
    public DcMotorEx intakeRoller;

    public CRServo intakePivot;
    public Servo intakeDoor;

    public AnalogInput intakePivotEnc;

    public ColorRangeSensor intakeCS;


    public static Hardware getInstance() {
        if (instance == null) {
            instance = new Hardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(final HardwareMap map) {
        hardwareMap = map;

        // Drivetrain
        LF = hardwareMap.get(DcMotorEx.class, "EH-Motor-0");
        RF = hardwareMap.get(DcMotorEx.class, "EH-Motor-1");
        RB = hardwareMap.get(DcMotorEx.class, "EH-Motor-2");
        LB = hardwareMap.get(DcMotorEx.class, "EH-Motor-3");

        pinPoint = hardwareMap.get(GoBildaPinpointDriver.class, "CH-I2C-0");

        // Deposit
        depositSlideRight = hardwareMap.get(DcMotorEx.class, "CH-Motor-0");
        depositSlideLeft = hardwareMap.get(DcMotorEx.class, "CH-Motor-1");

        armRight = hardwareMap.get(CRServo.class, "CH-Servo-0");
        armLeft = hardwareMap.get(CRServo.class, "CH-Servo-1");
        claw = hardwareMap.get(CRServo.class, "CH-Servo-2");

        armRightEnc = hardwareMap.get(AnalogInput.class, "CH-Analog-0");
        armLeftEnc = hardwareMap.get(AnalogInput.class, "CH-Analog-1");
        clawEnc = hardwareMap.get(AnalogInput.class, "CH-Analog-2");

        // Intake
        intakeSlide = hardwareMap.get(DcMotorEx.class, "CH-Motor-2");
        intakeRoller = hardwareMap.get(DcMotorEx.class, "CH-Motor-3");

        intakePivot = hardwareMap.get(CRServo.class, "CH-Servo-3");
        intakeDoor = hardwareMap.get(Servo.class, "CH-Servo-4");

        intakePivotEnc = hardwareMap.get(AnalogInput.class, "CH-Analog-3");

        intakeCS = hardwareMap.get(ColorRangeSensor.class, "CH-I2C-1");
    }

}
