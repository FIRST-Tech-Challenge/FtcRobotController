package org.firstinspires.ftc.teamcode.common.gps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.Kinematics.Kinematics;

import java.util.HashMap;

import java.io.FileWriter;   // Import the FileWriter class
import java.io.IOException;  // Import the IOException class to handle errors

public class SplineGPS {

    Constants constants = new Constants();
    Kinematics kinematics;

    private double[] positionArr = new double[4];
    private HashMap<DcMotorEx, Integer> motorClicksPose = new HashMap<>();
    private HashMap<DcMotorEx, Integer> prevMotorClicks = new HashMap<>();

    HardwareDrive robot;

    public SplineGPS(HardwareDrive robot){
        this.robot = robot;
        for (int i = 0; i < 4; i++){
            positionArr[i] = 0;
        }

        for (DcMotorEx motor : robot.dtMotors){
            motorClicksPose.put(motor, motor.getCurrentPosition());
            prevMotorClicks.put(motor, motor.getCurrentPosition());
        }
    }

    public void grabKinematics(Kinematics k){
        kinematics = k;
    }

    public void calculatePos(){
        if (!goodGap()) return;

        for (DcMotorEx motor : robot.dtMotors){
            prevMotorClicks.put(motor, motorClicksPose.get(motor));
            motorClicksPose.put(motor, motor.getCurrentPosition());
        }

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
//        double rotateR = rotateL;
//        double translateR = translateL;

        double rotationalDegrees = (rotateL + rotateR) / 2;
        double translationalInches = (translateL + translateR) / 2;

        double splineOrientation = 0.0;
        double otherAngle = rotationalDegrees;

        if (kinematics.getDriveType() == Kinematics.DriveType.SPLINE){
            double bigArc = Math.max(translateL, translateR); //unit: inches
            double smallArc = Math.min(translateL, translateR); //unit: inches
            double radius = ((bigArc + smallArc) * constants.DISTANCE_BETWEEN_MODULE_AND_CENTER) / (bigArc - smallArc); //unit: inches
            double theta = (bigArc - smallArc) / (2 * constants.DISTANCE_BETWEEN_MODULE_AND_CENTER); //unit: radians
            translationalInches = Math.sqrt((2 * radius * radius) * (1 - Math.cos(theta))); //value of hypotenuse, not arc. Ask Josh for more info.
            splineOrientation = Math.toDegrees(theta);

            otherAngle = (Math.PI - theta) / 2.0; //unit: radians
            otherAngle = (Math.PI / 2.0) - otherAngle;
        }

        if (kinematics.getDriveType() == Kinematics.DriveType.TURN){
            double radius = constants.DISTANCE_BETWEEN_MODULE_AND_CENTER;
            double arc = Math.abs(translateL + translateR)/2.0;
            double theta = arc / radius;
            theta = Math.toDegrees(theta);
            update(theta);
            return;
        }

        if (translationalInches == 0){
            update(translationalInches * Math.sin(otherAngle), translationalInches * Math.cos(otherAngle) , rotationalDegrees, 0);
        /*
        Problems:
        - completely breaks when robot rotates on its center (because the one of the arcs is reflected)
        To Do:
        - "Clamp" the output of orientations to (-180, 180], to keep it uniform with the rest of the program.
         */
        }
        else{
            update(translationalInches * Math.cos(otherAngle), translationalInches * Math.sin(otherAngle), splineOrientation, rotationalDegrees + splineOrientation);
        }
    }

    public void update ( double x, double y, double wheelR, double robotR){
        //update
        positionArr[0] += x * constants.INCHES_PER_CLICK;
        positionArr[1] += y * constants.INCHES_PER_CLICK;
        positionArr[2] += kinematics.clamp(wheelR * constants.DEGREES_PER_CLICK);
        positionArr[3] += kinematics.clamp(robotR * constants.DEGREES_PER_CLICK);
        //wth does "R" stand for in "wheelR" and "robotR" ?????????????
//        try {
//            FileWriter myWriter = new FileWriter("gpsLog.txt");
//            myWriter.write("GPS Log\n");
//            myWriter.write((int) positionArr[0] + "\n");
//            myWriter.write((int) positionArr[1] + "\n");
//            myWriter.write((int) positionArr[2] + "\n");
//            myWriter.write((int) positionArr[3] + "\n\n");
//            myWriter.close();
//        } catch (IOException e) {
//           // System.out.println("An error occurred.");
//            e.printStackTrace();
//        }
    }

    public void update(double robotR){
        positionArr[3] += kinematics.clamp(robotR * constants.DEGREES_PER_CLICK);
    }

    public void hardResetGPS(){
        //Reset GPS
        for (int i = 0; i < 4; i++){
            positionArr[i] = 0;
        }

        //Reset Motor Clicks
        for (DcMotorEx motor : robot.dtMotors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorClicksPose.put(motor, motor.getCurrentPosition());
            prevMotorClicks.put(motor, motor.getCurrentPosition());
        }
    }

    private boolean goodGap(){
        for (DcMotorEx motor : robot.dtMotors){
            if (Math.abs(motor.getCurrentPosition() - prevMotorClicks.get(motor)) <= constants.clickTOLERANCE) return false;
        }
        return true;
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

    public HashMap<DcMotorEx, Integer> getMotorClicksPose () {
        return motorClicksPose;
    }

}