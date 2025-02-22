package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
public class OneFishSampleDelivery {
    private DcMotor slide = null;
    private Servo claw = null;
    private Servo pitch = null;


    private LinearOpMode linearOpMode;

    private double currentSlidePower = 0;
    private double slidePowerMultiplier = 0.8;
    private double CLICKS_PER_METER = 2492.788;
    private double CLICKS_PER_INCH = 63.317;
    private final int MM_PER_METER = 1000;
    private int targetPosition;
    private double currentPosition;

    public final double TICK_STOP_THRESHOLD = 20;
    public final int BOTTOM_POSITION = (int)(2 / MM_PER_METER * CLICKS_PER_METER); //2mm
    public final int TOP_POSITION = 4300;
    public final int LOW_POSITION = 50; // low-to-ground position used to slow slides when low to ground

    public final int INTAKE_HEIGHT = 250;
    public final double GRAB_BUFFER_TIME = 350; //time for claw to grab specimen
    public final double LIFT_BUFFER_TIME = 250; //time for specimen to clear wall before moving
    public final double RESET_PITCH_BUFFER_TIME = 500; //time for pitching mechanism to make a full arc to target pitch
    public final double RESET_HEIGHT_BUFFER_TIME = 500; //time for slides to make a full ascent/descent to target pitch
    public final double SPECIMEN_SCORE_BUFFER_TIME = 500;
    public final int DELIVER_HEIGHT = 500;
    public final int CLEARANCE_HEIGHT = 500; //height where pitching will have 360 degree clearance

    public final double SPECIMEN_DELIVER_PITCH = 0.6;
    public final double SPECIMEN_INTAKE_PITCH = 0.0;
    public final double VERTICAL_PITCH = 0.4;

    public final double TRANSFER_PITCH = 0.83+0.1;
    public final double DELIVER_PITCH = 0.36;
    public final double AUTO_DELIVER_PITCH = 0.3;
    public final double AWAY_PITCH = 0.35;
    public final double SHAKE_PITCH = 0.15;
    public final double TICK_LOW_POWER_DISTANCE = 75;

    public final double RETRACT_TIMEOUT = 7;

    private boolean slidesRunToPosition;

    public boolean isDisabled = false;

    private double initAttempts = 0;

    private ElapsedTime time = new ElapsedTime();



    public OneFishSampleDelivery(LinearOpMode l, Boolean slidesRunToPosition)
    {
        linearOpMode = l;
        slidesRunToPosition = slidesRunToPosition;
        Initialize();
    }


    private void Initialize(){//4mm
        try {
            slide  = linearOpMode.hardwareMap.get(DcMotor.class, "deliverySlide");
            pitch = linearOpMode.hardwareMap.get(Servo.class, "deliveryPitch");
            claw = linearOpMode.hardwareMap.get(Servo.class, "claw");
            claw.scaleRange(0.0, 0.25);
            pitch.scaleRange(0.0, 1);
            slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            slide.setTargetPosition(0);

            if (slidesRunToPosition) {
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else {
                slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            //slide.setDirection(DcMotor.Direction.REVERSE);

        }catch(NullPointerException e){
            initAttempts++;
            linearOpMode.telemetry.addData("Couldn't find delivery.       Attempt: ", initAttempts);
            isDisabled = true;
        }
    }

    public void setSlidesTargetPosition(int clicks){
        targetPosition = clicks;
        slide.setTargetPosition(targetPosition);
    }

    public void setSlidesPower(double power){
        //if (Math.abs(power-currentSlidePower) > 0.03 || (power == 0 && currentSlidePower != 0)) {
            currentSlidePower = power;
            slide.setPower(currentSlidePower);
        //}
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
        pitch.setPosition(pitchPosition);
    }
    public void disableServoController(){
        pitch.getController().pwmDisable();
    }

    public void pitchToTransfer(){
        setPitch(TRANSFER_PITCH);
    }

    public void pitchToShake(){
        setPitch(SHAKE_PITCH);
    }

    public void pitchToDeliver(){
        setPitch(DELIVER_PITCH);
    }

    public void pitchToAutoDeliver(){
        setPitch(AUTO_DELIVER_PITCH);
    }

    public void pitchToAway(){
        setPitch(AWAY_PITCH);
    }

    public void pitchToSpecimenDeliver(){
        setPitch(SPECIMEN_DELIVER_PITCH);
    }

    public void pitchToSpecimenIntake(){
        setPitch(SPECIMEN_INTAKE_PITCH);
    }

    public void pitchToVertical() { setPitch(VERTICAL_PITCH); }

    public int getMotorPosition(){ return slide.getCurrentPosition(); }

    public double getMotorPositionInches(){ return getMotorPosition() / CLICKS_PER_INCH; }

    public int getMotorTargetPosition(){
        return targetPosition;
    }

    public double getMotorTargetPositionInches() { return (double)getMotorTargetPosition() / CLICKS_PER_INCH; }

    public DcMotor.RunMode getRunMode(){
        return slide.getMode();
    }
    public void setRunMode(DcMotor.RunMode mode){ slide.setMode(mode); }

    public double getPitchingServoPosition(){return pitch.getPosition();}

    public void PControlPower(double powerMultiplier){
        double error = targetPosition - slide.getCurrentPosition();
        double power = (Math.abs(error) / TICK_LOW_POWER_DISTANCE);

        power = Math.max(0.1, Math.min(1, power));

        setSlidesPower(power * slidePowerMultiplier * powerMultiplier);
        linearOpMode.telemetry.addData("Current Pos: ", getMotorPosition());
        linearOpMode.telemetry.addData("Target Pos: ", getMotorTargetPosition());
        linearOpMode.telemetry.addLine();linearOpMode.telemetry.addLine();linearOpMode.telemetry.addLine();
        linearOpMode.telemetry.addData("Power: ",power);
        linearOpMode.telemetry.addData("Error: ",error);
//        linearOpMode.telemetry.update();
    }

    public double getClawPosition() {return claw.getPosition();}
    public void setClawPosition(double p){ // 0-1
        claw.setPosition(p);
    }
    public void clawClose(){
        setClawPosition(1);
    }
    public void clawOpen(){
        setClawPosition(0);
    }

/*

    public void Lift(int ticksFromOutsideChassis){
        slidePowerMultiplier = 0.75;
        setSlidesTargetPosition(BOTTOM_POSITION);
        PControlPower();

        double error = targetPosition - slide.getCurrentPosition();
        error = Math.abs(error);

        //LIFT
        while(error <= TICK_STOP_THRESHOLD){

            error = targetPosition - slide.getCurrentPosition();
            error = Math.abs(error);

            PControlPower();
            linearOpMode.telemetry.addData("Error: ", error);
            linearOpMode.telemetry.update();
        }
        linearOpMode.sleep(1000);

        setSlidesTargetPosition(CARRIAGE_OUTSIDE_CHASSIS + ticksFromOutsideChassis - 150);
        while(leftError <= TICK_STOP_THRESHOLD
                &&
                rightError <= TICK_STOP_THRESHOLD){

            leftError = targetPosition - leftSlide.getCurrentPosition();
            rightError = targetPosition - rightSlide.getCurrentPosition();
            leftError = Math.abs(leftError);
            rightError = Math.abs(rightError);

            PControlPower();
            linearOpMode.telemetry.addData("Left Error: ", leftError);
            linearOpMode.telemetry.addData("right Error: ", rightError);
            linearOpMode.telemetry.update();
        }
    }


    public void Retract(){
        time.reset();
        wrist.setPosition(servoDodge);

        //Waiting for servo to move
        while(time.seconds() < 0.5){
            linearOpMode.telemetry.addLine("Waiting to Retract");
            linearOpMode.telemetry.update();
        }
        int tempTarget = leftSlide.getCurrentPosition();
        while(((leftSlide.getCurrentPosition() > 0 || rightSlide.getCurrentPosition() > 0) || time.seconds() > RETRACT_TIMEOUT) && tempTarget > CARRIAGE_DODGE){
            tempTarget -= 10;

            setSlidesTargetPosition(tempTarget);

            setSlidesPower(0.5);
        }
//        tempTarget = leftSlide.getCurrentPosition();
        setSlidesTargetPosition(tempTarget);
        while(leftSlide.getCurrentPosition() > CARRIAGE_DODGE){

        }
        time.reset();
        while(time.seconds() < 0.3){
            linearOpMode.telemetry.addLine("Waiting to Fully Retract");
            linearOpMode.telemetry.update();
        }
        wrist.setPosition(servoIn);
        time.reset();
        while(time.seconds() < 0.25){
            linearOpMode.telemetry.addLine("Waiting to Fully Retract");
            linearOpMode.telemetry.update();
        }
        tempTarget = leftSlide.getCurrentPosition();
        //RETRACT
        while((leftSlide.getCurrentPosition() > 0 || rightSlide.getCurrentPosition() > 0) || time.seconds() > RETRACT_TIMEOUT){
            tempTarget -= 5;

            setSlidesTargetPosition(tempTarget);

            setSlidesPower(0.25);

            if(leftSlide.getCurrentPosition() < CARRIAGE_DODGE){
                wrist.setPosition(servoIn);
            }
        }
        setSlidesPower(0);



        slidePowerMultiplier = 0.75;
    }
    */

    /*FIXME: Verify if this works*/
    public class SlideToHeightRR implements Action {
        private boolean initialized = false;
        private int targetHeight;

        public SlideToHeightRR(int TargetHeight) {
            targetHeight = TargetHeight;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                setSlidesTargetPosition(targetHeight);
                initialized = true;
            }

            double pos = slide.getCurrentPosition();
            packet.addLine("In RR action");
            packet.addLine("Claw height:");
            packet.put("liftPos", pos);
            if (Math.abs(pos - targetHeight) > 20) {
                PControlPower(3);
                return true;
            } else {
                setSlidesPower(0);
                return false;
            }
        }
    }
    public Action SlideToHeightAction(int heightInTicks) {
        return new OneFishSampleDelivery.SlideToHeightRR(heightInTicks);
    }

    public class IntakeSpecimenRR implements Action {
        private boolean initialized = false;
        private int targetHeight;
        private ElapsedTime timer = new ElapsedTime();

        public IntakeSpecimenRR() {
            targetHeight = INTAKE_HEIGHT;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                setSlidesTargetPosition(targetHeight);
                initialized = true;
            }

            double pos = slide.getCurrentPosition();
            packet.addLine("In RR action");
            packet.addLine("Claw height:");
            packet.put("liftPos", pos);
            if (timer.milliseconds() < GRAB_BUFFER_TIME) {
                targetHeight = INTAKE_HEIGHT;
                PControlPower(3);
                clawClose();
                return true;
            } else if (timer.milliseconds() < GRAB_BUFFER_TIME + LIFT_BUFFER_TIME) {
                clawClose();
                targetHeight = CLEARANCE_HEIGHT;
                PControlPower(3);
                return true;
            } else {
                setSlidesPower(0);
                pitchToSpecimenDeliver();
                return false;
            }
        }
    }
    public Action IntakeSpecimenAction() {
        return new OneFishSampleDelivery.IntakeSpecimenRR();
    }

    public class PrepIntakeSpecimenRR implements Action {
        private boolean initialized = false;
        private int targetHeight;
        private ElapsedTime timer = new ElapsedTime();

        public PrepIntakeSpecimenRR() {
            targetHeight = INTAKE_HEIGHT;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                setSlidesTargetPosition(targetHeight);
                initialized = true;
            }

            double pos = slide.getCurrentPosition();
            packet.addLine("In RR action");
            packet.addLine("Claw height:");
            packet.put("liftPos", pos);
            if (timer.milliseconds() < RESET_PITCH_BUFFER_TIME) {
                clawOpen();
                pitchToSpecimenIntake();
                return true;
            } else if (timer.milliseconds() < RESET_PITCH_BUFFER_TIME + RESET_HEIGHT_BUFFER_TIME) {
                clawOpen();
                PControlPower(3);
                return true;
            } else {
                setSlidesPower(0);
                return false;
            }
        }
    }
    public Action PrepIntakeSpecimenAction() {
        return new OneFishSampleDelivery.PrepIntakeSpecimenRR();
    }

    public class PrepDeliverSpecimenRR implements Action {

        private boolean initialized = false;
        private int targetHeight;
        private ElapsedTime timer = new ElapsedTime();

        public PrepDeliverSpecimenRR() {
            targetHeight = DELIVER_HEIGHT;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                setSlidesTargetPosition(targetHeight);
                initialized = true;
            }

            double pos = slide.getCurrentPosition();
            packet.addLine("In RR action");
            packet.addLine("Claw height:");
            packet.put("liftPos", pos);
            if (timer.milliseconds() < RESET_HEIGHT_BUFFER_TIME) {
                clawClose();
                setSlidesTargetPosition(targetHeight);
                PControlPower(3);
                return true;
            } else if (timer.milliseconds() < RESET_HEIGHT_BUFFER_TIME + RESET_PITCH_BUFFER_TIME) {
                clawClose();
                pitchToSpecimenDeliver();
                return true;
            } else {
                setSlidesPower(0);
                return false;
            }
        }
    }

    public Action PrepDeliverSpecimenAction() {
        return new OneFishSampleDelivery.PrepDeliverSpecimenRR();
    }

    public class DeliverSpecimenRR implements Action {

        private boolean initialized = false;
        private int targetHeight;
        private ElapsedTime timer = new ElapsedTime();

        public DeliverSpecimenRR() {
            targetHeight = DELIVER_HEIGHT;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                setSlidesTargetPosition(targetHeight);
                initialized = true;
            }

            double pos = slide.getCurrentPosition();
            packet.addLine("In RR action");
            packet.addLine("Claw height:");
            packet.put("liftPos", pos);
            if (timer.milliseconds() < SPECIMEN_SCORE_BUFFER_TIME) {
                clawClose();
                setSlidesTargetPosition(targetHeight);
                PControlPower(3);
                pitchToVertical();
                return true;
            } else if (timer.milliseconds() < SPECIMEN_SCORE_BUFFER_TIME + RESET_HEIGHT_BUFFER_TIME) {
                clawOpen();
                return true;
            } else {
                setSlidesPower(0);
                return false;
            }
        }
    }

    public Action DeliverSpecimenAction() {
        return new OneFishSampleDelivery.DeliverSpecimenRR();
    }


}
