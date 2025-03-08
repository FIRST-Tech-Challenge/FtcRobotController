package org.firstinspires.ftc.teamcode;


import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.Serializable;
import java.util.Arrays;


@Disabled
public class TwoFishIntake {
    private DcMotor extension = null;
    private DcMotor intake = null;
    private Servo pitch = null;
    private CRServo transfer = null;
    public TouchSensor limitSwitch = null;
//    private HuskyLens huskyLens = null;

    private OpMode opMode;

    private final double CLICKS_PER_CM = 24.92788;

    public final int DELTA_EXTENSION = 1000;
    public int minExtension = -999999999;
    public int maxExtension = DELTA_EXTENSION;
    public double downPitch = 0.5353;
    public double upPitch = 0.4506;

    private final int TICK_LOW_POWER_DISTANCE = 25;

    private double targetLengthCM;
    private int targetLength;
    private double currentPosition;

    public final double TICK_STOP_THRESHOLD = 5;

    public static String posessedSampleColor;

    private ElapsedTime time = new ElapsedTime();


    class TwoFishIntakeValues implements Serializable {

        double downPitch;
        double upPitch;

        public TwoFishIntakeValues( double downPitch,
                                    double upPitch
        ) {
            this.downPitch = downPitch;
            this.upPitch = upPitch;
        }
    }

    String file = "TwoFishDeliveryValues.txt";
    TwoFishIntakeValues twoFishIntakeValues = new TwoFishIntakeValues(
            downPitch,
            upPitch
    );


    //Color Sensor variables
    public static final double RED_SAMPLE_HUE = 25.0;
    public static final double YELLOW_SAMPLE_HUE = 80.0;
    public static final double BLUE_SAMPLE_HUE = 210.0;
    public static final double AMBIENT_HUE = 190;

    private static final int COLOR_SAMPLE_REPETITIONS = 3;

    ColorSensor colorSens = null;

    float hsv[] = {0,0,0};

    ElapsedTime colorTimer;


    public TwoFishIntake(OpMode l)
    {
        opMode = l;
        Initialize();
    }


    private void Initialize(){//4mm
        try {
            extension = opMode.hardwareMap.get(DcMotor.class, "extension");
            intake = opMode.hardwareMap.get(DcMotor.class, "intake");
            pitch = opMode.hardwareMap.get(Servo.class, "intakePitch");
            transfer = opMode.hardwareMap.get(CRServo.class, "intakeTransfer");
            colorSens = opMode.hardwareMap.get(ColorSensor.class, "intakeColor");

            extension.setTargetPosition(extension.getCurrentPosition());

            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            intake.setDirection(DcMotorSimple.Direction.REVERSE);
            extension.setDirection(DcMotorSimple.Direction.FORWARD);

            pitch.scaleRange(0.0, 1.0);

            colorTimer = new ElapsedTime();

            limitSwitch = opMode.hardwareMap.get(TouchSensor.class, "intakeTouch");

//            huskyLens = linearOpMode.hardwareMap.get(HuskyLens.class, "husky");
//            huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        }catch(NullPointerException e){
            opMode.telemetry.addLine("Couldn't find intake");
        }
    }


    public void setSlidesTargetPosition(int ticks){
        targetLength = ticks;
        targetLength = Math.min(maxExtension, Math.max(minExtension, targetLength));
        extension.setTargetPosition(targetLength);
    }

    public void toMinExtention(){setSlidesTargetPosition(minExtension);}

    public void setSlidesPower(double power){
        extension.setPower(power);
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
        runToPosition();
//        opMode.telemetry.addData("Intake Length: ", targetLength);
//        opMode.telemetry.addData("Intake Current: ", extension.getCurrentPosition());
        if(Math.abs(extension.getCurrentPosition() - extension.getTargetPosition()) > TICK_STOP_THRESHOLD){
            extension.setPower(1);
        } else{
            extension.setPower(0);
        }
    }
    
    public void setIntakePower(float power){intake.setPower(power);}
    public void setTransferPower(float power){transfer.setPower(power);}
    public void setPitch(float position){pitch.setPosition(position);}
    public void pitchUp(){pitch.setPosition(upPitch);}
    public void pitchDown(){pitch.setPosition(downPitch);}
    public double getPitch(){return pitch.getPosition();}
    public int getExtensionTicks(){ return extension.getCurrentPosition();}

    public void limitCheck() {
        if (limitSwitch.isPressed()) {
            minExtension = extension.getCurrentPosition();
            maxExtension = minExtension+DELTA_EXTENSION;
        }
    }

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
                posessedSampleColor = "RED";
            case(1):
                posessedSampleColor = "YELLOW";
            case(2):
                posessedSampleColor = "BLUE";
            default:
                posessedSampleColor = "NONE";
        }
        return posessedSampleColor;
    }

    public boolean verifyColor(String target){
        if(target == null) return false;
        if(getSampleColor().equals(target)){
            return true;
        }
        return false;
    }

    public String getSampleColor(){
        double[] hues = new double[COLOR_SAMPLE_REPETITIONS];
        int i = 0;
        hues = getHueRecursive(hues, i, COLOR_SAMPLE_REPETITIONS);
        Arrays.sort(hues);
        double median = hues[COLOR_SAMPLE_REPETITIONS/2];
        return getClosestHue(median);
    }
    
    private double[] getHueRecursive(double[] huesCopy, int index, int numReps){
        if(index >= numReps-1){
            return huesCopy;
        } else{
            if(index < numReps && colorTimer.milliseconds() > 10) {
                colorTimer.reset();
                index++;
                Color.RGBToHSV(colorSens.red(), colorSens.green(), colorSens.blue(), hsv);
                huesCopy[index] = hsv[0];
            }
            return getHueRecursive(huesCopy, index, numReps);
        }
    }

    private double[] getHue(int numReps){
        double[] huesTemp = new double[numReps];
        for(int i = 0; i < numReps; i++){
            if(colorTimer.milliseconds() < 10){
                i--;
            } else{
                Color.RGBToHSV(colorSens.red(), colorSens.green(), colorSens.blue(), hsv);
                huesTemp[i] = hsv[0];
            }
        }
        return huesTemp;
    }
}


