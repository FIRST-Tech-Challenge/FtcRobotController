package org.firstinspires.ftc.teamcode.common.gps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.ConstantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.Kinematics.Kinematics;

import java.util.HashMap;

import java.io.FileWriter;   // Import the FileWriter class
import java.io.IOException;  // Import the IOException class to handle errors

public class TurnGPS {

    Constants constants = new Constants();

    private double[] positionArr = new double[4];
    private HashMap<DcMotorEx, Integer> motorClicksPose = new HashMap<>();
    private HashMap<DcMotorEx, Integer> prevMotorClicks = new HashMap<>();

    HardwareDrive robot;

    private Kinematics.DriveType driveType;

    public double rotationalDegrees;
    public double translationalInches;

    public TurnGPS(HardwareDrive robot, Kinematics.DriveType k){
        this.robot = robot;
        for (int i = 0; i < 4; i++){
            positionArr[i] = 0;
        }

        updateHash();
    }

    public void calculatePos(){
        updateHash();

        //left
        int topL = motorClicksPose.get(robot.topL) - prevMotorClicks.get(robot.topL); //change in top left
        int botL = motorClicksPose.get(robot.botL) - prevMotorClicks.get(robot.botL); //change in bottom left
        double translateL = (topL - botL) / 2.0;
        double rotateL = topL - translateL;
        translateL *= constants.INCHES_PER_CLICK;
        rotateL *= constants.DEGREES_PER_CLICK;

        //right
        int topR = motorClicksPose.get(robot.topR) - prevMotorClicks.get(robot.topR); //change in top right
        int botR = motorClicksPose.get(robot.botR) - prevMotorClicks.get(robot.botR); //change in bottom right
        double translateR = (topR - botR) / 2.0;
        double rotateR = topR - translateR;
        translateR *= constants.INCHES_PER_CLICK;
        rotateR *= constants.DEGREES_PER_CLICK;

        rotationalDegrees = (rotateL + rotateR) / 2;
        translationalInches = (translateL + translateR) / 2;
        double currentAngle = clamp(rotationalDegrees + positionArr[2]);

        if (driveType == Kinematics.DriveType.TURN){
            double radius = constants.DISTANCE_BETWEEN_MODULE_AND_CENTER;
            double arc = Math.abs(translateL + translateR)/2.0;
            double theta = arc / radius;
            theta = Math.toDegrees(theta);
            update(theta);
            return;
        }

        if (Math.abs(translationalInches) <= 0.3){
            update(translationalInches * Math.sin(currentAngle), translationalInches * Math.cos(currentAngle) , rotationalDegrees, 0);
        }
        else{
            //note that while testing rotateGPS, ONLY wheel rotation should be changing.
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

    public void update (double robotR){
        //update
        positionArr[3] += robotR;
        positionArr[3] = clamp(positionArr[3]);
    }

    public boolean xChange(){
        return (Math.abs(positionArr[0]) <= 0.3);
    }

    public boolean yChange(){
        return (Math.abs(positionArr[1]) <= 0.3);
    }

    public boolean wChange(){
        return (Math.abs(positionArr[2]) <= 1);
    }

    public boolean rChange(){
        return (Math.abs(positionArr[3]) <= 1);
    }

    public void hardResetGPS(){
        //Reset GPS
        for (int i = 0; i < 4; i++){
            positionArr[i] = 0;
        }


        robot.topL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.botL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.topR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.botR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        updateHash();
    }

    public void updateHash(){
        motorClicksPose.put(robot.topR, robot.topR.getCurrentPosition());
        motorClicksPose.put(robot.botR, robot.botR.getCurrentPosition());
        motorClicksPose.put(robot.topL, robot.topL.getCurrentPosition());
        motorClicksPose.put(robot.botL, robot.botL.getCurrentPosition());

        prevMotorClicks.put(robot.topR, motorClicksPose.get(robot.topR));
        prevMotorClicks.put(robot.botR, motorClicksPose.get(robot.botR));
        prevMotorClicks.put(robot.topL, motorClicksPose.get(robot.topL));
        prevMotorClicks.put(robot.botL, motorClicksPose.get(robot.botL));
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
        return driveType;
    }

    public HashMap<DcMotorEx, Integer> getMotorClicksPose () {
        return motorClicksPose;
    }

}