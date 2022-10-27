package org.firstinspires.ftc.teamcode.common.gps;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.ConstantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.Kinematics.Kinematics;

import java.util.HashMap;

public class LinearGPS {

    Constants constants = new Constants();
    private Kinematics.DriveType driveType;

    private double[] positionArr = new double[4];
    public HashMap<String, Integer> motorClicksPose = new HashMap<>();
    public HashMap<String, Integer> prevMotorClicks = new HashMap<>();

    HardwareDrive robot;

    public double rotationalDegrees;
    public double translationalInches;

    public LinearGPS(HardwareDrive robot, Kinematics.DriveType k ){
        this.robot = robot;
        driveType = k;


        motorClicksPose.put("topR", robot.topR.getCurrentPosition());
        motorClicksPose.put("botR", robot.botR.getCurrentPosition());
        motorClicksPose.put("topL", robot.topL.getCurrentPosition());
        motorClicksPose.put("botL", robot.botL.getCurrentPosition());

        prevMotorClicks.put("topR", motorClicksPose.get("topR"));
        prevMotorClicks.put("botR", motorClicksPose.get("botR"));
        prevMotorClicks.put("topL", motorClicksPose.get("topL"));
        prevMotorClicks.put("botL", motorClicksPose.get("botL"));
    }

    public void calculatePos(){
//        if (goodGap()) return;

        updateHash();

        //left
        int topL = motorClicksPose.get("topL") - prevMotorClicks.get("topL"); //change in top left
        int botL = motorClicksPose.get("botL") - prevMotorClicks.get("botL"); //change in bottom left
        double translateL = (topL - botL) / 2.0;
        double rotateL = topL - translateL;
        translateL *= constants.INCHES_PER_CLICK;
        rotateL *= constants.DEGREES_PER_CLICK;

        //right
        int topR = motorClicksPose.get("topR") - prevMotorClicks.get("topR"); //change in top right
        int botR = motorClicksPose.get("botR") - prevMotorClicks.get("botR"); //change in bottom right
        double translateR = (topR - botR) / 2.0;
        double rotateR = topR - translateR;
        translateR *= constants.INCHES_PER_CLICK;
        rotateR *= constants.DEGREES_PER_CLICK;
//        double rotateR = rotateL;
//        double translateR = translateL;

        rotationalDegrees = (rotateL + rotateR) / 2;
        translationalInches = (translateL + translateR) / 2;
        double currentAngle = clamp(rotationalDegrees + positionArr[2]);
        currentAngle = Math.toRadians(currentAngle);

        if (Math.abs(translationalInches) <= 0.3){
            update(translationalInches * Math.sin(currentAngle), translationalInches * Math.cos(currentAngle) , rotationalDegrees, 0);
        }
        else{
            update(translationalInches * Math.cos(currentAngle), translationalInches * Math.sin(currentAngle), 0, rotationalDegrees);
        }
    }

    public void update ( double x, double y, double wheelR, double robotR){
        //update
        positionArr[0] += x;
        positionArr[1] += y;
        positionArr[2] += wheelR;
        positionArr[3] += robotR;
        positionArr[2] = clamp(positionArr[2]);
        positionArr[3] = clamp(positionArr[3]);
    }

    public boolean xChange(){
        return (Math.abs(positionArr[0]) <= 0.3);
    }

    public boolean yChange(){
        return (Math.abs(positionArr[1]) <= 0.3);
    }

    public boolean dChange(){
        return (xChange() || yChange());
    }

    public boolean wChange(){
        return (Math.abs(positionArr[2]) <= 1);
    }

    public boolean rChange(){
        return (Math.abs(positionArr[3]) <= 1);
    }

//    public void hardResetGPS(){
//        //Reset GPS
//        for (int i = 0; i < 4; i++){
//            positionArr[i] = 0;
//        }
//
//        //Reset Motor Clicks
//        for (DcMotorEx motor : robot.dtMotors){
//            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            motorClicksPose.put(motor, motor.getCurrentPosition());
//            prevMotorClicks.put(motor, motor.getCurrentPosition());
//        }
//    }

    public void updateHash(){
        prevMotorClicks.put("topR", motorClicksPose.get("topR"));
        prevMotorClicks.put("botR", motorClicksPose.get("botR"));
        prevMotorClicks.put("topL", motorClicksPose.get("topL"));
        prevMotorClicks.put("botL", motorClicksPose.get("botL"));

        motorClicksPose.put("topR", robot.topR.getCurrentPosition());
        motorClicksPose.put("botR", robot.botR.getCurrentPosition());
        motorClicksPose.put("topL", robot.topL.getCurrentPosition());
        motorClicksPose.put("botL", robot.botL.getCurrentPosition());
    }

//    private boolean goodGap(){
//        if ( //will need to add the left side later
//                Math.abs(robot.topR.getCurrentPosition() - prevMotorClicks.get(robot.topR)) <= constants.clickTOLERANCE && Math.abs(robot.botR.getCurrentPosition() - prevMotorClicks.get(robot.botR)) <= constants.clickTOLERANCE
//        ) {
//            goodGapw = false;
//            return false;
//        }
//
//        goodGapw = true;
//        return true;
//    }

    public double[] getPositionArr() {
        return positionArr;
    }

    public int[] getMotorClicks(){
        int[] clicks = new int[4];
        clicks[0] = robot.topL.getCurrentPosition();
        clicks[1] = robot.botL.getCurrentPosition();
        clicks[2] = robot.topR.getCurrentPosition();
        clicks[3] = robot.botR.getCurrentPosition();

        return clicks;
    }

    public double clamp(double degrees){
        if (Math.abs(degrees) >= 360) degrees %= 360;

        if (degrees < -179 || degrees > 180) {
            int modulo = (int)Math.signum(degrees) * -180;
            degrees = Math.floorMod((int)degrees, modulo);
        }
        return degrees;
    }

    public Kinematics.DriveType getDriveType() {
        return driveType;
    }

    public HashMap<String, Integer> getMotorClicksPose () {
        return motorClicksPose;
    }

}