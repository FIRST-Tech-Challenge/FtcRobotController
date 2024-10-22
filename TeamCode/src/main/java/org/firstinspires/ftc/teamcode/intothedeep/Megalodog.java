package org.firstinspires.ftc.teamcode.intothedeep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.chassis.MegalodogChassis;

public class Megalodog extends MegalodogChassis {
    private int extensionSliderMax = 3000;
    private int liftLowerBasket = 900;
    private int liftUpperBasket = 2800;
    private int liftLowerSpecimenBar = 500;
    private int liftUpperSpecimenBar = 2000;
    private int liftSnapSpecimen = 200;
    private int liftGetSpecimenFromWall = 500;
    private int liftHangOnLowerBar = 2000;
    private int liftHangOnUpperBar = 1000;
    private double extensionServoHome = 0.72;
    private double extensionServoDump = 0.1;
    private double deliveryServoHome = 0.3;
    private double deliveryServoDump = 0.95;
    private double specimenServoOpen = 0;
    private double specimenServoClosed = 0.8;
    private double continuousIntakePower = 0.8;
    private double extensionServoPosition;
    private double extensionServoSafetyPosition = 0.5;
    private double deliveryBoxServoPosition;
    private double specimenServoPosition;
    private DcMotor Lift;
    private TouchSensor ExtensionLimit;
    private TouchSensor LiftLimit;
    private TouchSensor LiftLimit2;
    private Servo ExtensionServo;
    private Servo DeliveryBoxServo;
    private CRServo IntakeBoxServo;
    private DcMotor ExtensionSlider;
    public Servo SpecimenGripperServo;
    private LinearOpMode myOpMode;
    private int extensionSliderPosition;



    public Megalodog(LinearOpMode currentOpMode)
    {
        super(currentOpMode);
        myOpMode = currentOpMode;
        HardwareMap hardwareMap = myOpMode.hardwareMap;
    }

    public void InitializeDevices()
    {

    }
    public void InitializePositions()
    {
        extensionServoPosition = extensionServoHome;
        ExtensionServo.setPosition(extensionServoPosition);
        deliveryBoxServoPosition = deliveryServoHome;
        DeliveryBoxServo.setPosition(deliveryServoHome);
        specimenServoPosition = specimenServoOpen;
        SpecimenGripperServo.setPosition(specimenServoPosition);
    }

    private void initializeDevices()
    {
        ExtensionServo = myOpMode.hardwareMap.get(Servo.class, "Extension");
        DeliveryBoxServo = myOpMode.hardwareMap.get(Servo.class, "DeliveryBox");
        IntakeBoxServo = myOpMode.hardwareMap.get(CRServo.class, "IntakeBox");
        SpecimenGripperServo = myOpMode.hardwareMap.get(Servo.class,"SpecimenGripper");
        ExtensionSlider = myOpMode.hardwareMap.get(DcMotor.class, "IntakeExtension");
        Lift = myOpMode.hardwareMap.get(DcMotor.class, "Lift");
        ExtensionLimit = myOpMode.hardwareMap.get(TouchSensor.class, "ExtensionLimit");
        LiftLimit = myOpMode.hardwareMap.get(TouchSensor.class, "LiftLimit");
        LiftLimit2 = myOpMode.hardwareMap.get(TouchSensor.class, "LiftLimit2");

        resetExtensionSlider(3000);

        resetLift(3000);
    }

    private void resetExtensionSlider(long safetyDuration)
    {

        ExtensionSlider.setDirection(DcMotor.Direction.FORWARD);
        ExtensionSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ExtensionSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        long startTime = System.currentTimeMillis(); // Record the start time
        long maxDuration = safetyDuration; // Maximum duration in milliseconds (3 seconds)

        while(!ExtensionLimit.isPressed()) {
            ExtensionSlider.setPower(-0.5);
            if (System.currentTimeMillis() - startTime > maxDuration) { break;}
        }
        ExtensionSlider.setPower(0);

        ExtensionSlider.setDirection(DcMotor.Direction.FORWARD);
        ExtensionSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ExtensionSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ExtensionSlider.setTargetPosition(0);
        ExtensionSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ExtensionSlider.setPower(0.4);
        extensionSliderPosition = 0;
    }

    private void resetLift(long safetyDuration)
    {

        Lift.setDirection(DcMotor.Direction.FORWARD);
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        long startTime = System.currentTimeMillis(); // Record the start time
        long maxDuration = safetyDuration; // Maximum duration in milliseconds (3 seconds)
        while(!LiftLimit.isPressed() && !LiftLimit2.isPressed()) {
            Lift.setPower(-0.5);
            if (System.currentTimeMillis() - startTime > maxDuration) { break;}
        }
        Lift.setPower(0);

        Lift.setDirection(DcMotor.Direction.FORWARD);
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setTargetPosition(0);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift.setPower(0.6);
    }

    public void TurnIntakeOn () {
//It turns the continues servo forward

    }
    public void TurnIntakeOff () {
//It turns the continues servo off

    }
    public void GrabSpeicen (int waitime) {
//The servo rotates forward

    }
    public void HookAndLetGo (int waitime) {
// It pushes the sepiecem down and then lets go of it

    }

    public void MoveSlideAndScoop (int distanceMM,int wait){};

    public void RaiseLift (int hightMM, int wait){};

    public void EmptyLift (int wait){};

    public void LetGoOfSpecimen(int wait){};

    public void CheckSampleColor (){};

    public void runintake(int direction,int howLong,int wait){
        //turn servo
    }
    public void RealeaseIntoBucket(int wait){
        //spin servo opposite
    }
    public void Returnlift(int wait){
        //turn motor
    }
    public void GrabandLift(int height, int wait){
        //turn servo and raise lift
    }
}

