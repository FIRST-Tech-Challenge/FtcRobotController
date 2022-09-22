package org.firstinspires.ftc.teamcode.ultimategoal2020.sensors2020;

import android.graphics.Color;
import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.ebotsenums.CoordinateSystem;
import org.firstinspires.ftc.teamcode.ultimategoal2020.FieldPosition2020;
import org.firstinspires.ftc.teamcode.ebotsenums.RobotSide;

import java.util.ArrayList;
import java.util.Formatter;

public class EbotsColorSensor implements EbotsSensor, EbotsSensorReading<EbotsColorSensor.TapeColor>{

    /***************************************************************
     ******    CLASS VARIABLES
     ***************************************************************/
    private float redColor;
    private float blueColor;
    private float greenColor;
    private float hue;
    private float value;
    private float alpha;
    private final float SCALE_FACTOR = 255;
    private final float[] hsvValues = new float[3];

    // Note:  observedColor is the returned object for getReading
    private TapeColor observedColor = null;  //Caution, may be null

    //Note:  There is no accumulated value, just return null
    private TapeColor accumulator = null;

    //private ColorSensor colorSensor;
    private NormalizedColorSensor colorSensor;
    public SensorLocation sensorLocation;

    private boolean debugOn = false;
    private String logTag = "Ebots";

    /*****************************************************************
     //******    Enumerations
     //****************************************************************/
    public enum TapeColor {
        RED (0, 30),
        BLUE (240, 30),
        WHITE(0,30);

        private float hueNom;
        private float hueTolerance;

        TapeColor(float hueNomIn, float hueTolIn){
            this.hueNom = hueNomIn;
            this.hueTolerance = hueTolIn;
        }

        public float getHueNom() {
            return hueNom;
        }

        public float getHueTolerance() {
            return hueTolerance;
        }

        public float getHueMin(){
            float hueMin = this.hueNom - this.hueTolerance;
            if(hueMin < 0) hueMin += 255;
            return hueMin;
        }

        public float getHueMax(){
            float hueMax = this.hueNom + this.hueTolerance;
            if(hueMax > 255) hueMax-=255;
            return hueMax;
        }

    }

    public enum SensorLocation {
        //Enum values(constructor arguments)
        FRONT_LEFT("frontLeftSensorColor",8,8),
        FRONT_RIGHT("frontRightSensorColor", 8,-8),
        BACK_LEFT("backLeftSensorColor", -8, 8),
        BACK_RIGHT("backRightSensorColor", -8, -8);

        //Enum (class) variables
        private String deviceName;
        private FieldPosition2020 sensorRobotPosition;

        //Enum Constructor
        SensorLocation(String deviceNameIn, double robotXPosition, double robotYPosition) {
            deviceName = deviceNameIn;
            sensorRobotPosition = new FieldPosition2020(robotXPosition, robotYPosition, CoordinateSystem.ROBOT);
        }
        //Enum Getters
        public String getDeviceName() {
            return deviceName;
        }
        public FieldPosition2020 getSensorRobotPosition(){return sensorRobotPosition;}
    }

    /***************************************************************
     ******    CONSTRUCTORS
     ***************************************************************/

    public EbotsColorSensor(SensorLocation sensorLocationIn, HardwareMap hardwareMap) {
        this.sensorLocation = sensorLocationIn;

        //this.colorSensor = hardwareMap.get(ColorSensor.class, sensorLocationIn.getDeviceName());
        float gain = 5*4;
        this.colorSensor = hardwareMap.get(NormalizedColorSensor.class, sensorLocationIn.getDeviceName());
        this.colorSensor.setGain(gain);

        if (this instanceof SwitchableLight) {
            ((SwitchableLight)this).enableLight(false);
        }
    }

    /*****************************************************************
     //******    INTERFACE METHODS FOR EbotsSensor
     //****************************************************************/
    @Override
    public void reset(){
        observedColor = null;
    }

    @Override
    public void performHardwareRead() {
//        redColor = (int) (colorSensor.red() * SCALE_FACTOR);
//        blueColor = (int) (colorSensor.blue() * SCALE_FACTOR);
//        greenColor = (int) (colorSensor.green() * SCALE_FACTOR);

        // Get the normalized colors from the sensor
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        redColor = colors.red * SCALE_FACTOR;
        blueColor = colors.blue * SCALE_FACTOR;
        greenColor = colors.green * SCALE_FACTOR;

        // Update the hsvValues array by passing it to Color.colorToHSV()
        Color.colorToHSV(colors.toColor(), hsvValues);
        this.hue = hsvValues[0];
        this.value = hsvValues[2];
        this.alpha = colors.alpha;

        // set the local variable observedColor based on the sensor reading
        this.observedColor = getObservedTapeColor();

    }

    @Override
    public void flushReading(){
        // There is nothing to move to the accumulator
        this.reset();
    }

    @Override
    public TapeColor getReading() {
        return this.observedColor;
    }

    @Override
    public void performErrorCheck() {
        //  No error checking is implemented yet
    }


    /*****************************************************************
     //******    SIMPLE GETTERS AND SETTERS
     //****************************************************************/
    public TapeColor getObservedColor() { return observedColor;  }

    /*****************************************************************
     //******    CLASS STATIC METHODS
     //****************************************************************/

    public static EbotsColorSensor getEbotsColorSensor(SensorLocation sensorLocation, ArrayList<EbotsColorSensor> ebotsColorSensors) {
        EbotsColorSensor returnSensor = null;
        for (EbotsColorSensor sensor : ebotsColorSensors) {
            if (sensorLocation == sensor.sensorLocation) {
                returnSensor = sensor;
                break;
            }
        }
        return returnSensor;
    }

    public static boolean isSideOnColor(ArrayList<EbotsColorSensor> ebotsColorSensors, RobotSide robotSide, TapeColor tapeColor){
        boolean returnValue = true;     //assumes both wheels are on the color
        ArrayList<SensorLocation> sensorLocations = getSensorLocationsForSide(robotSide);  //find which wheel locations are on a side
        //Loop through all color sensors
        for(EbotsColorSensor ecs: ebotsColorSensors) {
            if (sensorLocations.contains(ecs.sensorLocation)){  //If on the side
                //if (!ecs.isColor(tapeColor)){ //replace this calculation with property value
                if(!(ecs.observedColor == tapeColor)){      //If not the expected color
                    returnValue = false;                    //return false
                }
            }
        }
        return returnValue;
    }

    private static ArrayList<SensorLocation> getSensorLocationsForSide(RobotSide robotSide) {
        ArrayList<SensorLocation> sensorLocations = new ArrayList<>();
        if (robotSide == RobotSide.FRONT) {
            sensorLocations.add(SensorLocation.FRONT_LEFT);
            sensorLocations.add(SensorLocation.FRONT_RIGHT);
        } else if (robotSide == RobotSide.LEFT) {
            sensorLocations.add(SensorLocation.FRONT_LEFT);
            sensorLocations.add(SensorLocation.BACK_LEFT);
        } else if (robotSide == RobotSide.RIGHT) {
            sensorLocations.add(SensorLocation.FRONT_RIGHT);
            sensorLocations.add(SensorLocation.BACK_RIGHT);
        } else if (robotSide == RobotSide.BACK) {
            sensorLocations.add(SensorLocation.BACK_RIGHT);
            sensorLocations.add(SensorLocation.BACK_LEFT);
        }
        return sensorLocations;
    }



    /*****************************************************************
     //******    CLASS INSTANCE METHODS
     //****************************************************************/
    public boolean isRed() {
        boolean returnValue = false;
//        int redThreshold = 170;
//        int redOther = 100;
//        if (redColor >= redThreshold && blueColor <= redOther && greenColor <= redOther) {
//            returnValue = true;
//        }
        returnValue = this.isObservationInHueRange(TapeColor.RED);
        return returnValue;
    }

    public boolean isBlue() {
        boolean returnValue = false;
//        int blueThreshold = 170;
//        int blueOther = 100;
//        if (blueColor >= blueThreshold && redColor <= blueOther && greenColor <= blueOther) {
//            returnValue = true;
//        }
        returnValue = this.isObservationInHueRange(TapeColor.BLUE);
        return returnValue;
    }

    public boolean isWhite() {
        boolean returnValue = false;
//        int whiteThreshold = 200;
//        if (redColor >= whiteThreshold && blueColor >= whiteThreshold && greenColor >= whiteThreshold) {
//            returnValue = true;
//        }
        float valueMIN = 0.6f;
        if(this.value > valueMIN) returnValue = true;

        if(debugOn) Log.d(logTag, this. sensorLocation.toString() + " is WHITE" + "? Observed value: "
                + String.format("%.2f", value) + " greater than " + String.format("%.2f", valueMIN)
                +  " ? " + returnValue);

        return returnValue;
    }

    public boolean isColor(TapeColor tapeColor) {
        boolean returnValue = false;
        if(tapeColor == TapeColor.WHITE) {
            returnValue = this.isWhite();
        } else{
            returnValue = this.isObservationInHueRange(tapeColor);
        }
//        }else if(tapeColor == TapeColor.BLUE){
//            returnValue = this.isBlue();
//
//        }else if(tapeColor == TapeColor.RED){
//            returnValue = this.isRed();
//        }
        return returnValue;
    }

    public boolean isObservationInHueRange(TapeColor tapeColor){
        boolean returnValue = false;
        float hueMin = tapeColor.getHueMin();
        float hueMax = tapeColor.getHueMax();
        float observedHue = this.hue;

        if(hueMin < hueMax){
            //Observation must be between both values if no hue wrapping
            if(observedHue >= hueMin && observedHue <= hueMax){
                returnValue = true;
            }
        } else{
            //If color wraps, then must use OR logic
            //In this case, the hue MIN number is actually larger value
            if((observedHue >= hueMin && observedHue < 255)
                    | observedHue <= hueMax && observedHue > 0
            ){
                returnValue = true;
            }
        }

        if(debugOn) Log.d(logTag, this. sensorLocation.toString() + " is " + tapeColor.toString() + "? Observed hue: "
                + String.format("%.2f", hue) + " in range [" + String.format("%.2f", hueMin)
                + ", " + String.format("%.2f", hueMax) + "] ? "
                + returnValue);

        return returnValue;
    }

    private TapeColor getObservedTapeColor(){
        TapeColor returnColor = null;
        for(TapeColor tc: TapeColor.values()){
            if(isColor(tc)){
                returnColor = tc;
                break;
            }
        }
        return returnColor;
    }

    public static String printColorsObserved(ArrayList<EbotsColorSensor> ebotsColorSensors){

        StringBuilder sb = new StringBuilder();
        sb.append("FRONT: ");
        EbotsColorSensor sensor = getEbotsColorSensor(SensorLocation.FRONT_LEFT, ebotsColorSensors);
        sb.append((sensor.observedColor != null) ? sensor.observedColor.toString(): "BLACK");
        sb.append(" | ");
        sensor = getEbotsColorSensor(SensorLocation.FRONT_RIGHT, ebotsColorSensors);
        sb.append((sensor.observedColor != null) ? sensor.observedColor.toString(): "BLACK");
        sb.append("\nBACK : ");
        sensor = getEbotsColorSensor(SensorLocation.BACK_LEFT, ebotsColorSensors);
        sb.append((sensor.observedColor != null) ? sensor.observedColor.toString(): "BLACK");
        sb.append(" | ");
        sensor = getEbotsColorSensor(SensorLocation.BACK_RIGHT, ebotsColorSensors);
        sb.append((sensor.observedColor != null) ? sensor.observedColor.toString(): "BLACK");
        return sb.toString();
    }

    @Override
    public String toString(){
        StringBuilder sb = new StringBuilder();
        Formatter fmt = new Formatter(sb);
        sb.append(this.sensorLocation.toString());
        sb.append("R: ");
        fmt.format("%.3f", this.redColor);
        sb.append(", G: ");
        fmt.format("%.3f", this.greenColor);
        sb.append(", B: ");
        fmt.format("%.3f", this.blueColor);
        sb.append(" -- h:");
        fmt.format("%.3f", hsvValues[0]);
        sb.append(", s:");
        fmt.format("%.3f", hsvValues[1]);
        sb.append(", v:");
        fmt.format("%.3f", hsvValues[2]);
        sb.append(", a:");
        fmt.format("%.3f", this.alpha);

        return sb.toString();
    }



}
