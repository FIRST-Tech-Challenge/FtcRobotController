package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

@Disabled
public class TwoFishDelivery {
    private static final double TICK_LOW_POWER_DISTANCE = 200;
    private DcMotor slide = null;
    private Servo claw = null;
    private Servo pitch1 = null;
    private Servo pitch2 = null;
    private Servo wrist = null;
    private VoltageSensor voltageSensor = null;
    //    private TouchSensor limitSwitch = null;


    private LinearOpMode linearOpMode;

    private double CLICKS_PER_METER = 2492.788;
    private double CLICKS_PER_INCH = 63.317;
    private final int MM_PER_METER = 1000;
    private boolean slidesRunToPosition;

    public boolean isDisabled = false;

    private double initAttempts = 0;

    private ElapsedTime time = new ElapsedTime();

    public double slideTarget;

    public double clawOpenPosition = 0;
    public double clawClosePosition = 1;
    public double pitchIntakePosition = 1;
    public double pitchUpPosition = 0.5;
    public double pitchScoreSpecPosition = 0.25;
    public double pitchTransferPosition = 0;
    public double pitchSampleScorePosition = 0.75;

    public double wristUpPosition = 1;
    public double wristDownPosition = 0;
    private double operatingVoltage;
    final private double DIP_FROM_HOLDING_SPEC = 0.25;
    final private double DIP_FROM_SCORING_SPEC = 0.5;

    //Slide Heights
    final private int SPEC_HEIGHT = 350;
    final private int SAMPLE_HEIGHT = 350;


    //boolean:
    private boolean clawClosed = false;

    //action timestamps:
    private double clawCloseTimestamp = 0;
    private double clawOpenTimestamp = 0;

    //action durations:
    private double CLAW_CLOSE_DURATION = 0.5;


    String file = "TwoFishDeliveryValues.txt";
    TwoFishDeliveryValues twoFishDeliveryValues = new TwoFishDeliveryValues(
            clawOpenPosition,
            clawClosePosition,
            pitchIntakePosition,
            pitchUpPosition,
            pitchTransferPosition,
            pitchScoreSpecPosition,
            wristUpPosition,
            wristDownPosition
    );

    public TwoFishDelivery(LinearOpMode l, ElapsedTime deliveryTimer)
    {
        time = deliveryTimer;
        linearOpMode = l;
        Initialize();
    }


    private void Initialize(){//4mm


        try {
            voltageSensor = linearOpMode.hardwareMap.get(VoltageSensor.class, "Control Hub");
            operatingVoltage = voltageSensor.getVoltage();

            slide  = linearOpMode.hardwareMap.get(DcMotor.class, "deliverySlide");
            pitch1 = linearOpMode.hardwareMap.get(Servo.class, "deliveryPitchLeft");
            pitch2 = linearOpMode.hardwareMap.get(Servo.class, "deliveryPitchRight");
            claw = linearOpMode.hardwareMap.get(Servo.class, "claw");
            wrist = linearOpMode.hardwareMap.get(Servo.class, "wrist");
//          limitSwitch = linearOpMode.hardwareMap.get(TouchSensor.class, "deliveryTouch");

            claw.scaleRange(0.0, 1);
            pitch1.scaleRange(0.0, 1);
            pitch2.scaleRange(0.0, 1);
            pitch2.setDirection(Servo.Direction.REVERSE);
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            slide.setTargetPosition(0);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


//            FileOutputStream fileOut = new FileOutputStream(file);
//
//            // Creates an ObjectOutputStream
//            ObjectOutputStream objOut = new ObjectOutputStream(fileOut);
//
//            // Writes objects to the output stream
//            objOut.writeObject("Hi");
//
//            // Reads the object
//            FileInputStream fileIn = new FileInputStream(file);
//            ObjectInputStream objIn = new ObjectInputStream(fileIn);
//
//            // Reads the objects
//            //Dog newDog = (Dog) objIn.readObject();
//
//            System.out.println("Claw Open: " + newDog.name);
//            System.out.println("Dog Breed: " + newDog.breed);
//
//            objOut.close();
//            objIn.close();


            //slide.setDirection(DcMotor.Direction.REVERSE);

        }catch(NullPointerException e){
            initAttempts++;
            linearOpMode.telemetry.addData("Couldn't find delivery.       Attempt: ", initAttempts);
            isDisabled = true;
        }
    }

    public void resetPWM(){
        pitch1.getController().pwmDisable();
    }

    public void setSlidesTargetPosition(int clicks){
        slideTarget = clicks;
    }

    public void setSlidesPower(double power){
        slide.setPower(power);
    }

    public void manualMove(double power){
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setPower(power);
    }
    public void setSlidesRunToPosition(){
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void resetEncoder(){
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setPitch(double pitchPosition){
        pitch1.setPosition(pitchPosition);
        pitch2.setPosition(pitchPosition);
    }
    public void setWrist(double rollPosition){
        wrist.setPosition(rollPosition);
    }


    public int getMotorPosition(){ return slide.getCurrentPosition(); }

    public double getMotorPositionInches(){ return getMotorPosition() / CLICKS_PER_INCH; }


    public DcMotor.RunMode getRunMode(){
        return slide.getMode();
    }
    public void setRunMode(DcMotor.RunMode mode){ slide.setMode(mode); }


    public void PControlPower(double powerMultiplier){
        double error = slideTarget - slide.getCurrentPosition();
        double power = (Math.abs(error) / TICK_LOW_POWER_DISTANCE);

        power = Math.max(0.1, Math.min(1, power));

        setSlidesPower(power * powerMultiplier);
    }

    public void toSpecHeight(){
        setSlidesTargetPosition(SPEC_HEIGHT);
    }
    public void toSampleHeight(){
        setSlidesTargetPosition(SAMPLE_HEIGHT);
    }
    public void setClawPosition(double p){ // 0-1
        claw.setPosition(p);
    }
    public void clawClose(){
        clawCloseTimestamp = time.seconds();
        setClawPosition(1);
        clawClosed = true;
    }
    public void clawOpen(){
        setClawPosition(0);
        clawClosed = false;
    }

    public boolean CheckClawClosed(){
        return (time.seconds() - clawCloseTimestamp > CLAW_CLOSE_DURATION && clawClosed);
    }
    public boolean CheckClawOpen(){
        return (time.seconds() - clawOpenTimestamp > CLAW_CLOSE_DURATION && !clawClosed);
    }

    public boolean checkIfHoldingSpec(){
        return voltageSensor.getVoltage() < operatingVoltage - DIP_FROM_HOLDING_SPEC;
    }

    public boolean checkIfScoredSpec(){
        return voltageSensor.getVoltage() < operatingVoltage - DIP_FROM_SCORING_SPEC;
    }

//    public void limitCheck() {
//        if (limitSwitch.isPressed()) {
//            minExtension = extension.getCurrentPosition();
//            maxExtension = minExtension+DELTA_EXTENSION;
//        }
//    }

    public void addServoTelemetry(){
        linearOpMode.telemetry.addData("pitch L: ", pitch1.getPosition());
        linearOpMode.telemetry.addData("pitch R: ", pitch2.getPosition());

        linearOpMode.telemetry.addData("claw: ", claw.getPosition());
        linearOpMode.telemetry.addData("wrist: ", wrist.getPosition());
    }

    public void addSlideTelemetry(){
        linearOpMode.telemetry.addData("Slide Target: ", slideTarget);
        linearOpMode.telemetry.addData("Slide Current: ", slide.getCurrentPosition());
    }

}

class TwoFishDeliveryValues implements Serializable {

    double clawOpenPosition;
    double clawClosePosition;

    double pitchIntakePosition;
    double pitchUpPosition;
    double pitchTransferPosition;
    double pitchScorePosition;

    double wristUp;
    double wristDown;

    public TwoFishDeliveryValues(double clawOpenPosition,
                                 double clawClosePosition,
                                 double pitchIntakePosition,
                                 double pitchUpPosition,
                                 double pitchTransferPosition,
                                 double pitchScorePosition,
                                 double wristUp,
                                 double wristDown
    ) {
        this.clawOpenPosition = clawOpenPosition;
        this.clawClosePosition = clawClosePosition;

        this.pitchIntakePosition = pitchIntakePosition;
        this.pitchTransferPosition = pitchTransferPosition;
        this.pitchUpPosition = pitchUpPosition;
        this.pitchScorePosition = pitchScorePosition;

        this.wristUp = wristUp;
        this.wristDown = wristDown;
    }
}
