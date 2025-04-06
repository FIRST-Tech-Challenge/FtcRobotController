package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.Serializable;

@Disabled
public class TwoFishDelivery {
    private static final double TICK_LOW_POWER_DISTANCE = 200;
    private DcMotor slide0 = null;
    private DcMotor slide1 = null;
    private Servo claw = null;
    private Servo pitch1 = null;
    private Servo pitch2 = null;
    private Servo wrist = null;
    private VoltageSensor voltageSensor = null;
        public TouchSensor limitSwitch = null;


    private OpMode opMode;

    private double CLICKS_PER_METER = 2492.788;
    private double CLICKS_PER_INCH = 63.317;
    private final int MM_PER_METER = 1000;
    private boolean slidesRunToPosition;

    public boolean isDisabled = false;

    private double initAttempts = 0;

    private ElapsedTime time = new ElapsedTime();

    public double slideTarget;

    public double clawOpenPosition = 0.3289;
    public double clawClosePosition = 0.6622;
    public double pitchIntakePosition = 1;
    public double pitchUpPosition = 0.57;
    public double pitchScoreSpecPosition = 0.29;
    public double pitchTransferPosition = 0;
    public double pitchSampleScorePosition = 0.7683;

    public double wristUpPosition = 0;
    public double wristDownPosition = 0.6311;
    private double operatingVoltage;
    final private double DIP_FROM_HOLDING_SPEC = 0.25;
    final private double DIP_FROM_SCORING_SPEC = 0.5;

    //Slide Heights
    final private int SPEC_DELTA = 480;
    final private int SAMPLE_DELTA = 3500;

    public int minHeight = -999999999;
    public int maxHeight = 3250;
    public int specHeight = SPEC_DELTA;
    public int sampleHeight = SAMPLE_DELTA;

    //boolean:
    public boolean clawClosed = false;
    public boolean isPitching = false;

    //action timestamps:
    private double clawCloseTimestamp = 0;
    private double clawOpenTimestamp = 0;
    private double clawClearTimestamp = 0;
    private double specScoreTimestamp = 0;

    //action durations:
    private double CLAW_CLOSE_DURATION = 0.5;

    private double SPEC_SCORE_DURATION = 0.75;


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

    public TwoFishDelivery(OpMode l, ElapsedTime deliveryTimer)
    {
        time = deliveryTimer;
        opMode = l;
        Initialize();
    }


    private void Initialize(){//4mm


        try {
            voltageSensor = opMode.hardwareMap.get(VoltageSensor.class, "Control Hub");
            operatingVoltage = voltageSensor.getVoltage();

            slide0 = opMode.hardwareMap.get(DcMotor.class, "deliverySlide0");
            slide1 = opMode.hardwareMap.get(DcMotor.class, "deliverySlide1");
            pitch1 = opMode.hardwareMap.get(Servo.class, "deliveryPitchLeft");
            pitch2 = opMode.hardwareMap.get(Servo.class, "deliveryPitchRight");
            claw = opMode.hardwareMap.get(Servo.class, "claw");
            wrist = opMode.hardwareMap.get(Servo.class, "wrist");
          limitSwitch = opMode.hardwareMap.get(TouchSensor.class, "deliveryTouch");

            claw.scaleRange(0.0, 1);
            pitch1.scaleRange(0.0, 1);
            pitch2.scaleRange(0.0, 1);
            pitch2.setDirection(Servo.Direction.REVERSE);

            slide1.setDirection(DcMotorSimple.Direction.REVERSE);


            slide0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            slide0.setTargetPosition(0);
            slide1.setTargetPosition(0);
            slide0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


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
            opMode.telemetry.addData("Couldn't find delivery.       Attempt: ", initAttempts);
            isDisabled = true;
        }
    }

    public void resetPWM(){
        pitch1.getController().pwmDisable();
    }

    public void setSlidesTargetPosition(int clicks){
        slideTarget = clicks;
        slideTarget = Math.min(maxHeight, Math.max(minHeight, slideTarget));
        slide0.setTargetPosition((int)slideTarget);
        slide1.setTargetPosition((int)slideTarget);
    }

    public void setSlidesPower(double power){
        slide0.setPower(power);
        slide1.setPower(power);
    }

    public void manualMove(double power){
        slide0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide0.setPower(power);
        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide1.setPower(power);
    }
    public void setSlidesRunToPosition(){
        slide0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void resetEncoder(){
        slide0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setPitch(double pitchPosition){
        clawClearTimestamp = time.seconds();
        specScoreTimestamp = time.seconds();
        pitch1.setPosition(pitchPosition);
       pitch2.setPosition(pitchPosition);

        isPitching = true;
    }
    public double getPitch(){
        return pitch1.getPosition();
    }

    public boolean CheckIfDonePitching(){
        if(time.seconds() - clawClearTimestamp > CLAW_CLOSE_DURATION && isPitching){
            isPitching = false;
            return true;
        }
        return false;
    }

    public boolean CheckIfDonePitching(double duration){
        if(time.seconds() - clawClearTimestamp > duration && isPitching){
            isPitching = false;
            return true;
        } else{
            return false;
        }
    }
//    public boolean CheckIfDoneScoringSpec(){
//        if(time.seconds() - specScoreTimestamp > SPEC_SCORE_DURATION && isPitching){
//            isPitching = false;
//        }
//        return (time.seconds() - specScoreTimestamp > SPEC_SCORE_DURATION && isPitching);
//    }
    public void setWrist(double rollPosition){
        wrist.setPosition(rollPosition);
    }
    public double getWrist(){return wrist.getPosition();}



    public int getMotorPosition(){ return slide0.getCurrentPosition(); }

    public double getMotorPositionInches(){ return getMotorPosition() / CLICKS_PER_INCH; }


    public DcMotor.RunMode getRunMode(){
        return slide0.getMode();
    }
    public void setRunMode(DcMotor.RunMode mode){
        slide0.setMode(mode);
        slide1.setMode(mode);
    }


    public void PControlPower(double powerMultiplier){
        double error = slideTarget - slide0.getCurrentPosition();
        double power = (Math.abs(error) / TICK_LOW_POWER_DISTANCE);

        power = Math.max(0.1, Math.min(1, power));

        setSlidesPower(power * powerMultiplier);
    }
    public int getSlideHeight(){
        return slide0.getCurrentPosition();
    }

    public void toMinHeight(){setSlidesTargetPosition(minHeight);}
    public void toSpecHeight(){setSlidesTargetPosition(specHeight);}
    public void toTransferHeight(){setSlidesTargetPosition(minHeight + 80);}
    public void toSampleHeight(){
        setSlidesTargetPosition(sampleHeight);
    }
    public void setClawPosition(double p){ // 0-1
        claw.setPosition(p);
    }
    public void clawClose(){
        clawCloseTimestamp = time.seconds();
        setClawPosition(clawClosePosition);
        clawClosed = true;
    }
    public void clawOpen(){
        clawOpenTimestamp = time.seconds();
        setClawPosition(clawOpenPosition);
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

    public void stopSlidesIfStuck(){
        if(voltageSensor.getVoltage() < operatingVoltage - 1){
            slide0.setPower(0);
            slide1.setPower(0);
        }
    }

    public void limitCheck() {
        if (limitSwitch.isPressed()) {
            minHeight = slide0.getCurrentPosition();
            specHeight = minHeight+SPEC_DELTA;
            sampleHeight = minHeight+SAMPLE_DELTA;
        }
    }

    public void addServoTelemetry(){
        opMode.telemetry.addData("pitch L: ", pitch1.getPosition());
        opMode.telemetry.addData("pitch R: ", pitch2.getPosition());

        opMode.telemetry.addData("claw: ", claw.getPosition());
        opMode.telemetry.addData("wrist: ", wrist.getPosition());
    }

    public void addSlideTelemetry(){
        opMode.telemetry.addData("Slide Target: ", slideTarget);
        opMode.telemetry.addData("Slide0 Current: ", slide0.getCurrentPosition());
        opMode.telemetry.addData("Slide1 Current: ", slide1.getCurrentPosition());
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
