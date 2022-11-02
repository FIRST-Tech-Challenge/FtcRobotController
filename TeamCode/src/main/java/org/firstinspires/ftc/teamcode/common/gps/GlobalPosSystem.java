package org.firstinspires.ftc.teamcode.common.gps;

import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.kinematics.Kinematics;

import java.util.HashMap;

public class GlobalPosSystem {

    Constants constants = new Constants();
    Kinematics kinematics;

    private double[] positionArr = new double[5];
    public HashMap<String, Integer> motorClicksPose = new HashMap<>();
    public HashMap<String, Integer> prevMotorClicks = new HashMap<>();

    HardwareDrive robot;

    public GlobalPosSystem(HardwareDrive robot){
        this.robot = robot;

        motorClicksPose.put("topR", robot.topR.getCurrentPosition());
        motorClicksPose.put("botR", robot.botR.getCurrentPosition());
        motorClicksPose.put("topL", robot.topL.getCurrentPosition());
        motorClicksPose.put("botL", robot.botL.getCurrentPosition());

        prevMotorClicks.put("topR", motorClicksPose.get("topR"));
        prevMotorClicks.put("botR", motorClicksPose.get("botR"));
        prevMotorClicks.put("topL", motorClicksPose.get("topL"));
        prevMotorClicks.put("botL", motorClicksPose.get("botL"));
    }

    public void grabKinematics(Kinematics k){
        kinematics = k;
    }

    public void calculatePos(){
        updateHash();

        //right
        int topR = motorClicksPose.get("topR") - prevMotorClicks.get("topR"); //change in top right
        int botR = motorClicksPose.get("botR") - prevMotorClicks.get("botR"); //change in bottom right
        double translationalInchesR = (topR - botR) / 2.0;
        double rotationalDegreesR = topR - translationalInchesR;
        translationalInchesR *= constants.INCHES_PER_CLICK;
        rotationalDegreesR *= constants.DEGREES_PER_CLICK;

        double currentAngleR = clamp(rotationalDegreesR + positionArr[3]);
        //left
        int topL = motorClicksPose.get("topL") - prevMotorClicks.get("topL"); //change in top left
        int botL = motorClicksPose.get("botL") - prevMotorClicks.get("botL"); //change in bottom left
        double translationalInchesL = (topL - botL) / 2.0;
        double rotationalDegreesL = topL - translationalInchesL;
        translationalInchesL *= constants.INCHES_PER_CLICK;
        rotationalDegreesL *= constants.DEGREES_PER_CLICK;

        double currentAngleL = clamp(rotationalDegreesL + positionArr[2]);

        double splineOrientation = 0.0;
        double baseAngle = (currentAngleL + currentAngleR) / 2.0;
        baseAngle = Math.toRadians(baseAngle);
        double hypotenuse = (translationalInchesL + translationalInchesR) / 2.0;

//        double bigArc = Math.max(translationalInchesL, translationalInchesR); //unit: inches
//        double smallArc = Math.min(translationalInchesL, translationalInchesR); //unit: inches
//        if (Math.abs(bigArc - smallArc) <= 0.1){
//            double radius = ((bigArc + smallArc) * constants.DISTANCE_BETWEEN_MODULE_AND_CENTER) / (bigArc - smallArc); //unit: inches
//            double theta = (bigArc - smallArc) / (2 * constants.DISTANCE_BETWEEN_MODULE_AND_CENTER); //unit: radians
//            hypotenuse = Math.sqrt((2 * radius * radius) * (1 - Math.cos(theta)));
//            splineOrientation = Math.toDegrees(theta);
//            baseAngle = (Math.PI - theta) / 2.0; //unit: radians
//            baseAngle = (Math.PI / 2.0) - baseAngle;
//        } //problem: this assumes that the modules are parallel.

        if (Math.abs(hypotenuse) <= 0.20){
            update(hypotenuse * Math.sin(baseAngle), hypotenuse * Math.cos(baseAngle), rotationalDegreesL, rotationalDegreesR, 0);
        }
        else{
            double tableSpin = (rotationalDegreesL + rotationalDegreesR) / 2.0;
            update(hypotenuse * Math.sin(baseAngle), hypotenuse * Math.cos(baseAngle), splineOrientation, splineOrientation, splineOrientation + tableSpin);
        }
    }

    public void update ( double x, double y, double leftWheelW, double rightWheelW, double robotR){
        //update
        positionArr[0] += x;
        positionArr[1] += y;
        positionArr[2] += leftWheelW;
        positionArr[3] += rightWheelW;
        positionArr[4] += robotR;

        positionArr[2] = clamp(positionArr[2]);
        positionArr[3] = clamp(positionArr[3]);
        positionArr[4] = clamp(positionArr[4]);
    }

    public double getLeftWheelW(){
        return positionArr[2];
    }

    public double getRightWheelW(){
        return positionArr[3];
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

    private boolean goodGap(){
        return (Math.abs(robot.topR.getCurrentPosition() - prevMotorClicks.get("topR")) > constants.clickTOLERANCE || Math.abs(robot.botR.getCurrentPosition() - prevMotorClicks.get("botR")) > constants.clickTOLERANCE);
    }

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
        return kinematics.getDriveType();
    }

    public HashMap<String, Integer> getMotorClicksPose () {
        return motorClicksPose;
    }

}