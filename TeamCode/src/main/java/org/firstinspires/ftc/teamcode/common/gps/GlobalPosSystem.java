package org.firstinspires.ftc.teamcode.common.gps;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.ConstantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.Kinematics.Kinematics;

import java.util.HashMap;

public class GlobalPosSystem {

    Constants constants = new Constants();
    Kinematics kinematics;

    private double[] positionArr = new double[4];
    public HashMap<String, Integer> motorClicksPose = new HashMap<>();
    public HashMap<String, Integer> prevMotorClicks = new HashMap<>();

    HardwareDrive robot;

    public boolean goodGapw = true;

    public GlobalPosSystem(HardwareDrive robot){
        this.robot = robot;

        motorClicksPose.put("topR", robot.topR.getCurrentPosition());
        motorClicksPose.put("botR", robot.botR.getCurrentPosition());
//        motorClicksPose.put("topL", robot.topL.getCurrentPosition());
//        motorClicksPose.put("botL", robot.botL.getCurrentPosition());

        prevMotorClicks.put("topR", motorClicksPose.get("topR"));
        prevMotorClicks.put("botR", motorClicksPose.get("botR"));
//        prevMotorClicks.put("topL", motorClicksPose.get("topL"));
//        prevMotorClicks.put("botL", motorClicksPose.get("botL"));
    }

    public void grabKinematics(Kinematics k){
        kinematics = k;
    }

    public void calculatePos(){
        if (!goodGap()) return;

        updateHash();

        //right
        int topR = motorClicksPose.get("topR") - prevMotorClicks.get("topR"); //change in top right
        int botR = motorClicksPose.get("botR") - prevMotorClicks.get("botR"); //change in bottom right
        double translateR = (topR - botR) / 2.0;
        double rotateR = topR - translateR;
        translateR *= constants.INCHES_PER_CLICK;
        rotateR *= constants.DEGREES_PER_CLICK;
//        double rotateR = rotateL;
//        double translateR = translateL;


        //left
//        int topL = motorClicksPose.get("topL") - prevMotorClicks.get("topL"); //change in top left
//        int botL = motorClicksPose.get("botL") - prevMotorClicks.get("botL"); //change in bottom left
//        double translateL = (topL - botL) / 2.0;
//        double rotateL = topL - translateL;
//        translateL *= constants.INCHES_PER_CLICK;
//        rotateL *= constants.DEGREES_PER_CLICK;
        double translateL = translateR;
        double rotateL = rotateR;


        double rotationalDegrees = (rotateL + rotateR) / 2.0;
        double translationalInches = (translateL + translateR) / 2.0;
        double currentAngle = clamp(rotationalDegrees + positionArr[2]);
        currentAngle = Math.toRadians(currentAngle);

        double splineOrientation = 0;

        if (Math.abs(translationalInches) == 0){
            update(translationalInches * Math.sin(currentAngle), translationalInches * Math.cos(currentAngle) , rotationalDegrees, 0);
        }
        else{
            update(translationalInches * Math.cos(currentAngle), translationalInches * Math.sin(currentAngle), splineOrientation, rotationalDegrees + splineOrientation);
        }
    }

//    public void tempCalculateSpline(){
//        double splineOrientation = 0.0;
//        double otherAngle = rotationalDegrees;
//
//        if (kinematics.getDriveType() == Kinematics.DriveType.SPLINE){
//            double bigArc = Math.max(translateL, translateR); //unit: inches
//            double smallArc = Math.min(translateL, translateR); //unit: inches
//            double radius = ((bigArc + smallArc) * constants.DISTANCE_BETWEEN_MODULE_AND_CENTER) / (bigArc - smallArc); //unit: inches
//            double theta = (bigArc - smallArc) / (2 * constants.DISTANCE_BETWEEN_MODULE_AND_CENTER); //unit: radians
//            translationalInches = Math.sqrt((2 * radius * radius) * (1 - Math.cos(theta))); //value of hypotenuse, not arc. Ask Josh for more info.
//            splineOrientation = Math.toDegrees(theta);
//
//            otherAngle = (Math.PI - theta) / 2.0; //unit: radians
//            otherAngle = (Math.PI / 2.0) - otherAngle;
//        }
//    }

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

    public void setGoodGapw(boolean t){
        goodGapw = t;
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
//        prevMotorClicks.put("topL", motorClicksPose.get("topL"));
//        prevMotorClicks.put("botL", motorClicksPose.get("botL"));

        motorClicksPose.put("topR", robot.topR.getCurrentPosition());
        motorClicksPose.put("botR", robot.botR.getCurrentPosition());
//        motorClicksPose.put("topL", robot.topL.getCurrentPosition());
//        motorClicksPose.put("botL", robot.botL.getCurrentPosition());
    }

    private boolean goodGap(){
        return (Math.abs(robot.topR.getCurrentPosition() - prevMotorClicks.get("topR")) > constants.clickTOLERANCE || Math.abs(robot.botR.getCurrentPosition() - prevMotorClicks.get("botR")) > constants.clickTOLERANCE);
    }

    public double[] getPositionArr() {
        return positionArr;
    }

    public int[] getMotorClicks(){
        int[] clicks = new int[4];
//        clicks[0] = robot.topL.getCurrentPosition();
//        clicks[1] = robot.botL.getCurrentPosition();
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
        return kinematics.getDriveType();
    }

    public HashMap<String, Integer> getMotorClicksPose () {
        return motorClicksPose;
    }

}