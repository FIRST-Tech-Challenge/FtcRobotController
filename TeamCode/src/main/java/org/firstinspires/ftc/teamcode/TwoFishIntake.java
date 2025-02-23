package org.firstinspires.ftc.teamcode;


import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;


@Disabled
public class TwoFishIntake {
    private DcMotor extension = null;
    private DcMotor intake = null;
    private Servo pitch = null;
    private Servo transfer = null;
//    private TouchSensor limitSwitch = null;
//    private HuskyLens huskyLens = null;

    private LinearOpMode linearOpMode;

    public final double CLICKS_PER_DEGREE = 3.5;
    private final double CLICKS_PER_CM = 24.92788;
    private final int MM_PER_METER = 1000;

    public final int DELTA_EXTENSION = 1000;
    public int minExtension = -999999999;
    public int maxExtension = DELTA_EXTENSION;
    private final double MIN_PITCH = 0.5;
    private final double DOWN_PITCH = 1;
    private final double UP_PITCH = 0.5;
    private final double AWAY_PITCH = 0.0;
    private final double TRANSFER_PITCH = 0.09;
    private final double MAX_PITCH = 1.0;

    private final int TICK_LOW_POWER_DISTANCE = 50;

    private double targetLengthCM;
    private int targetLength;
    private double currentPosition;

    public final double TICK_STOP_THRESHOLD = 5;

    private ElapsedTime time = new ElapsedTime();




    //Color Sensor variables
    public static final double RED_SAMPLE_HUE = 25.0;
    public static final double YELLOW_SAMPLE_HUE = 80.0;
    public static final double BLUE_SAMPLE_HUE = 210.0;
    public static final double AMBIENT_HUE = 190;

    private static final int COLOR_SAMPLE_REPETITIONS = 3;

    ColorSensor colorSens;

    float hsv[] = {0,0,0};

    ElapsedTime colorTimer;



    public TwoFishIntake(LinearOpMode l)
    {
        linearOpMode = l;
        Initialize();
    }


    private void Initialize(){//4mm
        try {
            extension = linearOpMode.hardwareMap.get(DcMotor.class, "extension");
            extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            intake = linearOpMode.hardwareMap.get(DcMotor.class, "intake");
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            pitch = linearOpMode.hardwareMap.get(Servo.class, "intakePitch");
            pitch.scaleRange(0.5, 0.815);

            extension.setTargetPosition(extension.getCurrentPosition());

//            limitSwitch = linearOpMode.hardwareMap.get(TouchSensor.class, "intakeTouch");

//            huskyLens = linearOpMode.hardwareMap.get(HuskyLens.class, "husky");
//            huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        }catch(NullPointerException e){
            linearOpMode.telemetry.addLine("Couldn't find intake");
        }
    }


    public void setTargetLength(int ticks){
        targetLength = ticks;
        extension.setTargetPosition(Math.max(minExtension, Math.min(maxExtension, ticks)));
    }

    public void setExtensionPower(double power){
        int error = 0;
        int errorMult = 0;
        int current = extension.getCurrentPosition();
        extension.setPower(power);

        if(current > maxExtension /2){
            error = (current - maxExtension);
        } else {
            error = (current- minExtension);
        }

        errorMult = error/(TICK_LOW_POWER_DISTANCE*2);

        if(Math.abs(error) < TICK_LOW_POWER_DISTANCE){
            extension.setPower(power * errorMult);
            linearOpMode.telemetry.addLine("BRAKING");
        }
        linearOpMode.telemetry.addData("Intake Current: ", current);
    }

    public void runToPosition(){
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runPower(){
        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoder(){
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void updateLength(){
        linearOpMode.telemetry.addData("Intake Length: ", targetLength);
        linearOpMode.telemetry.addData("Intake Current: ", extension.getCurrentPosition());
        if(Math.abs(extension.getCurrentPosition() - extension.getTargetPosition()) > TICK_STOP_THRESHOLD){
            extension.setPower(1);
        } else{
            extension.setPower(0);
        }
    }


    public void setIntakePower(float power){
        intake.setPower(power);
    }

    public void setPitch(float position){
        pitch.setPosition(position);
    }
    public double getPitch(){return pitch.getPosition();}

    public void pitchUp(){
        pitch.setPosition(UP_PITCH);
    }
    public void pitchDown(){
        pitch.setPosition(DOWN_PITCH);
    }
    public void pitchAway(){pitch.setPosition(AWAY_PITCH);}

    public void pitchToTransfer(){pitch.setPosition(TRANSFER_PITCH);}

    public int getExtensionTicks(){ return extension.getCurrentPosition();}

//    public void limitCheck() {
//        if (limitSwitch.isPressed()) {
//            minExtension = extension.getCurrentPosition();
//            maxExtension = minExtension+DELTA_EXTENSION;
//        }
//    }

//    ArrayList<ArrayList<HuskyLens.Block>> blocks = new ArrayList<ArrayList<HuskyLens.Block>>(4);
//    //yellow blocks -> get(1)
//    //red blocks    -> get(2)
//    //blue blocks   -> get(3)
//
//    final int centerX = 160;
//    final int centerY = 120;
//    public void getHuskyDetections() {
//        HuskyLens.Block[] huskyBlocks = huskyLens.blocks();
//        linearOpMode.telemetry.addData("Block count", huskyBlocks.length);
//        for (HuskyLens.Block sample : huskyBlocks) {
//            blocks.get(sample.id).add(sample);
//        }
//    }
//
//    int distanceToCenter (HuskyLens.Block block) {
//        return (int) Math.sqrt(Math.pow(block.x - centerX,2) + Math.pow(block.y - centerY,2));
//    }
//    public int[] getCenterBlockDistance(String color) {
//        int blockColor = -1;
//        if (color == "yellow") {
//            blockColor = 1;
//        } else if (color == "red") {
//            blockColor = 2;
//        } else if (color == "blue") {
//            blockColor = 3;
//        }
//        Collections.sort(blocks.get(blockColor), new Comparator<HuskyLens.Block>(){
//            public int compare(HuskyLens.Block a, HuskyLens.Block b){
//                return distanceToCenter(a) - distanceToCenter(b);
//            }
//        });
//
//        int deltaX = blocks.get(blockColor).get(0).x-centerX;
//        int deltaY = blocks.get(blockColor).get(0).y-centerY;
//
//        return new int[] {deltaX/320, deltaY/240};
//    }

    public class RunToLengthRR implements Action {
        private boolean initialized = false;
        private int target = 0;
        //timeout in MILLISECONDS
        private double timeout = 0.0;

        private ElapsedTime timer = new ElapsedTime();

        public RunToLengthRR(int targetLength, double timeoutTime) {
            target = targetLength;
            timeout = timeoutTime;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
                setTargetLength(target);
                timer.reset();
            }

            packet.addLine("In RR action");
            packet.addLine("Drive For Time");
            packet.put("Time Elapsed", timer.milliseconds());
            if (Math.abs(extension.getCurrentPosition() - target) > 10 && timer.milliseconds() < timeout) {
                updateLength();
                return true;
            } else {
                setTargetLength(extension.getCurrentPosition());
                return false;
            }
        }
    }
    public Action RunToLengthAction(int targetLength, double timeoutTime) {
        return new TwoFishIntake.RunToLengthRR(targetLength, timeoutTime);
    }


    public class SpinIntakeRR implements Action {
        private boolean initialized = false;
        //timeout in MILLISECONDS
        private double timeout = 0.0;

        private double power = 0.0;

        private ElapsedTime timer = new ElapsedTime();

        public SpinIntakeRR(double spinPower, double timeoutTime) {
            timeout = timeoutTime;
            power = spinPower;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
                setIntakePower((float)power);
                timer.reset();
            }

            packet.addLine("In RR action");
            packet.addLine("Drive For Time");
            packet.put("Time Elapsed", timer.milliseconds());
            if (timer.milliseconds() < timeout) {
                return true;
            } else {
                setIntakePower(0);
                return false;
            }
        }
    }
    public Action SpinIntakeAction(double spinPower, double timeoutTime) {
        return new TwoFishIntake.SpinIntakeRR(spinPower, timeoutTime);
    }


    private String getClosestHue(double measuredHue){
        final double[] COLOR_ERRORS = {Math.abs(measuredHue-RED_SAMPLE_HUE), Math.abs(measuredHue-YELLOW_SAMPLE_HUE), Math.abs(measuredHue-BLUE_SAMPLE_HUE), Math.abs(measuredHue-AMBIENT_HUE)};
//      {redError, yellowError, blueError, ambientError}

        double min = Double.MAX_VALUE;
        int minIndex = Integer.MAX_VALUE;
        for(int i = 0; i < COLOR_ERRORS.length; i++){
            if(COLOR_ERRORS[i] < min){
                min = COLOR_ERRORS[i];
                minIndex = i;
            }
        }
        switch (minIndex){
            case(0):
                return "RED";
            case(1):
                return "YELLOW";
            case(2):
                return "BLUE";
            default:
                return "NONE";
        }

    }

    public String getSampleColor(){
        double[] hues = new double[COLOR_SAMPLE_REPETITIONS];
        int i = 0;
        hues = getColorHueRecursive(hues, i, COLOR_SAMPLE_REPETITIONS);
        Arrays.sort(hues);
        double median = hues[COLOR_SAMPLE_REPETITIONS/2];
        return getClosestHue(median);
    }

    private double[] getColorHueRecursive(double[] huesCopy, int index, int numReps){
        if(index >= numReps-1){
            return huesCopy;
        } else{
            if(index < numReps && colorTimer.milliseconds() > 10) {
                colorTimer.reset();
                index++;
                Color.RGBToHSV(colorSens.red(), colorSens.green(), colorSens.blue(), hsv);
                huesCopy[index] = hsv[0];
            }
            return getColorHueRecursive(huesCopy, index, numReps);
        }
    }
}


