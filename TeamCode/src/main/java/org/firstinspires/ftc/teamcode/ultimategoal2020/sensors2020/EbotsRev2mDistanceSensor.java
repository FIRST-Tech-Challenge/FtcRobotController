package org.firstinspires.ftc.teamcode.ultimategoal2020.sensors2020;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ebotsenums.RobotSide;

import java.util.ArrayList;
import java.util.Formatter;

public class EbotsRev2mDistanceSensor implements EbotsSensor, EbotsSensorReading<Double> {

    private Rev2mDistanceSensor rev2mDistanceSensor;
    private RobotSide robotSide;
    private double distanceInches;

    public enum DistanceSensorName {
        //Enum for mapping sensors to the hardwareMap using name
        //This is a bit redundant because assuming one per RobotSide
//        F("frontDistSensor"),
        R("rightDistSensor"),
        B("backDistSensor"),
        L("leftDistSensor");

        private String deviceName;

        DistanceSensorName(String deviceNameIn){
            this.deviceName = deviceNameIn;
        }

        //This is the primary function that is used
        public static String getSensorName(RobotSide robotSide){
            String returnName;
//            if(robotSide == RobotSide.FRONT) {
//                returnName = DistanceSensorName.F.getDeviceName();
//            } else
            if(robotSide == RobotSide.RIGHT){
                returnName = DistanceSensorName.R.getDeviceName();
            }else if(robotSide == RobotSide.BACK){
                returnName = DistanceSensorName.B.getDeviceName();
            }else {
                returnName = DistanceSensorName.L.getDeviceName();
            }
            return returnName;
        }

        public String getDeviceName() {
            return deviceName;
        }
    }

    public EbotsRev2mDistanceSensor(RobotSide robotSide, HardwareMap hardwareMap){
        this.robotSide = robotSide;
        this.rev2mDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, DistanceSensorName.getSensorName(robotSide));
    }

    public Rev2mDistanceSensor getRev2mDistanceSensor() {
        return rev2mDistanceSensor;
    }

    public RobotSide getRobotSide() {
        return robotSide;
    }
    public double getDistanceInches() {return this.distanceInches;}

    @Deprecated
    public void setDistanceInches(){
        //this should only be read once per cycle to improve control loop speed
        this.performHardwareRead();
    }


    /*
            IMPLEMENT INTERFACE METHODS
     */
    @Override
    public void reset() {
        distanceInches = 0;
    }

    @Override
    public void performHardwareRead() {
        //this should only be read once per cycle to improve control loop speed
        this.distanceInches = this.rev2mDistanceSensor.getDistance(DistanceUnit.INCH);
    }

    @Override
    public void flushReading() {
        // No accumulator action
        this.reset();
    }

    @Override
    public void performErrorCheck() {
        // Not yet implemented
    }

    @Override
    public Double getReading() {
        return this.distanceInches;
    }


    public static double getDistanceForRobotSide(RobotSide robotSide, ArrayList<EbotsRev2mDistanceSensor> ebotsRev2mDistanceSensors){
        EbotsRev2mDistanceSensor sensor = null;
        //Get the sensor located at the requested based on robotSide
        for(EbotsRev2mDistanceSensor ds: ebotsRev2mDistanceSensors){
            if(ds.robotSide == robotSide){
                sensor = ds;
                break;
            }
        }
        return (sensor != null) ? sensor.distanceInches : 0;
    }

    @Override
    public String toString(){
        return this.robotSide.toString() + " dist: " + String.format("%.2f", this.distanceInches + " in");
    }

    public static String printAll(ArrayList<EbotsRev2mDistanceSensor> dsList){
        StringBuilder sb = new StringBuilder();
        Formatter fmt = new Formatter(sb);
        sb.append("[L | R - ");
        fmt.format("%.2f", getDistanceForRobotSide(RobotSide.LEFT, dsList));
        sb.append(" | ");
        fmt.format("%.2f", getDistanceForRobotSide(RobotSide.RIGHT, dsList));
        sb.append(" ] [F | B - ");
//        fmt.format("%.2f", getDistanceForRobotSide(RobotSide.FRONT, dsList));
//        sb.append(" | ");
        fmt.format("%.2f", getDistanceForRobotSide(RobotSide.BACK, dsList));
        sb.append(" ]");
        return sb.toString();
    }
}
