package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

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

    List<LynxModule> hubs;

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

    public RevColorSensorV3 intakeCS;



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
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LF.setDirection(DcMotorSimple.Direction.REVERSE);

        RF = hardwareMap.get(DcMotorEx.class, "EH-Motor-1");
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RB = hardwareMap.get(DcMotorEx.class, "EH-Motor-2");
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LB = hardwareMap.get(DcMotorEx.class, "EH-Motor-3");
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);

        pinPoint = hardwareMap.get(GoBildaPinpointDriver.class, "CH-I2C-0-1");
        pinPoint.setOffsets(-71,-109.18776);
        pinPoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinPoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinPoint.resetPosAndIMU();

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub: hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }


        // Deposit
        depositSlideRight = hardwareMap.get(DcMotorEx.class, "CH-Motor-0");
        depositSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        depositSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        depositSlideLeft = hardwareMap.get(DcMotorEx.class, "CH-Motor-1");
        depositSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armRight = hardwareMap.get(CRServo.class, "CH-Servo-0");
        armLeft = hardwareMap.get(CRServo.class, "CH-Servo-1");
        claw = hardwareMap.get(CRServo.class, "CH-Servo-2");

        armRightEnc = hardwareMap.get(AnalogInput.class, "CH-Analog-0");
        armLeftEnc = hardwareMap.get(AnalogInput.class, "CH-Analog-1");
        clawEnc = hardwareMap.get(AnalogInput.class, "CH-Analog-2");

        // Intake
        intakeSlide = hardwareMap.get(DcMotorEx.class, "CH-Motor-2");
        intakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeRoller = hardwareMap.get(DcMotorEx.class, "CH-Motor-3");
        intakeRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intakePivot = hardwareMap.get(CRServo.class, "CH-Servo-3");
        intakeDoor = hardwareMap.get(Servo.class, "CH-Servo-4");

        intakePivotEnc = hardwareMap.get(AnalogInput.class, "CH-Analog-3");

        intakeCS = hardwareMap.get(RevColorSensorV3.class, "CH-I2C-1-0");
    }

    public void clearCache() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }

}
