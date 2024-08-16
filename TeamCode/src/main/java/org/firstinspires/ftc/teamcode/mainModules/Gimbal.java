package org.firstinspires.ftc.teamcode.mainModules;  //place where the code is located

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Gimbal {
    private AnalogInput potentiometer = null;
    private Servo gimbalPitch = null;
    private Servo gimbalYaw = null;
    private Servo gimbalPos = null;

    SlowUpDate pitchUpDate;
    SlowUpDate yawUpDate;

    private double wantedPitch = 1; //initial deafult values
    private double wantedYaw = 0.9;

    private Telemetry telemetry;

    public void initGimbal(HardwareMap hardwareMapPorted, Telemetry telemetryPorted) {

        pitchUpDate = new SlowUpDate();
        pitchUpDate.initSlowUpDate(10);
        yawUpDate = new SlowUpDate();
        yawUpDate.initSlowUpDate(10);


        telemetry = telemetryPorted;

        potentiometer = hardwareMapPorted.get(AnalogInput.class, "Analog_Port_0_CH");
        gimbalPitch = hardwareMapPorted.get(Servo.class, "Servo_Port_0_CH");
        gimbalYaw = hardwareMapPorted.get(Servo.class, "Servo_Port_1_CH");
        gimbalPos = hardwareMapPorted.get(Servo.class, "Servo_Port_2_CH");
    }

    public double AnglePosition() {return (potentiometer.getVoltage() * 81.8);} // 3.3v, 270 degrees
    

    public void telemetryGimbal() {
        telemetry.addData("Potentiometer Angle", AnglePosition());
    }

    public void untuck() {
        /*gimbalPos.setPosition(0.71);
        gimbalPitch.setPosition(0.91);
        gimbalYaw.setPosition(0.71);*/
        wantedYaw = 0.71;
        wantedPitch = 0.91;
    }

    public void tuck() {
        /*
        gimbalPos.setPosition(0.9);
        gimbalPitch.setPosition(1);
        gimbalYaw.setPosition(0.9);*/
        wantedYaw = 0.9;
        wantedPitch = 1;

    }
    public void moveGimbal(boolean automatic, boolean moveManual, double manualX,
                           double manualY, double ftcPoseX, double ftcPoseY, double ftcPoseZ, boolean upDateAutomatic){
        telemetry.addData("automaticGimbal", automatic);
        telemetry.addData("moveManual", moveManual);
        if (automatic) {

            wantedYaw = calculateServoAngle(ftcPoseX, ftcPoseZ, potentiometer.getVoltage());
            if (upDateAutomatic) {
                double checkNaN = Math.atan(ftcPoseY / ftcPoseZ);
                if (!Double.isNaN(checkNaN)){
                    wantedPitch = Math.atan(ftcPoseY / ftcPoseZ);
                }
            }
        }else{

            if (pitchUpDate.isTurn() ) {
                wantedPitch -= manualY/100;
            }

            if (yawUpDate.isTurn()) {
                wantedYaw += manualX/100;
            }

            /*
            gimbalPos.setPosition(0.5 + manualX / 2);
            gimbalYaw.setPosition(0.5 + manualX / 2);
            //gimbalPitch.setPosition(0.5 + manualY / 2);*/

        }
            /*
            double yawPosition = calculateServoAngle(ftcPoseX, ftcPoseZ, potentiometer.getVoltage());
            double pitchPosition = Math.atan(ftcPoseY/ftcPoseZ);

            gimbalYaw.setPosition(yawPosition);
            gimbalPos.setPosition(yawPosition);

            if (upDateAutomatic && !Double.isNaN(pitchPosition)){
                gimbalPitch.setPosition(pitchPosition);
            }*/

        wantedYaw = normalize(wantedYaw);
        wantedPitch = normalize(wantedPitch);
        gimbalPos.setPosition(wantedYaw);
        gimbalPitch.setPosition(wantedPitch);
        gimbalYaw.setPosition(wantedYaw);


    } private double normalize (double tooBig){
        if (tooBig<0){
            return 0;
        } if (tooBig>1){
            return 1;
        }
        return tooBig;
    }

    private double calculateServoAngle (double xy, double z, double potMeter) {

        double angleDifference = Math.atan(z/xy);
        telemetry.addData("xy", xy);
        telemetry.addData("z", z);
        telemetry.addData("angle-difference", angleDifference);
        telemetry.addData("potentiometer.getVoltage()", potMeter);
        double returnAngle = (1 - potMeter/3.3) + angleDifference/(Math.PI*3);
        telemetry.addData("returnAngle", returnAngle);
        if (Double.isNaN(returnAngle)){
            return potentiometer.getVoltage()/3.3;
        } else {
            return returnAngle;
        }



    }

}

class SlowUpDate {
    // System.currentTimeMillis(); return milliseconds long so everything is in long to avoid type conflicts
    private long msBetween = 100;
    private long lastTime = System.currentTimeMillis();

    void initSlowUpDate(long msBetween){
        this.msBetween = msBetween;
    }

    boolean isTurn(){
        long currentDiff = System.currentTimeMillis() - lastTime;
        if (currentDiff > msBetween) {
            lastTime = System.currentTimeMillis();
            return true;
        } else {
            return false;
        }
    }
}