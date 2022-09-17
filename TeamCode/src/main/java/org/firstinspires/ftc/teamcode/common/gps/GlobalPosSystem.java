package org.firstinspires.ftc.teamcode.common.gps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.internal.android.dx.util.IntIterator;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.Kinematics;

import java.util.HashMap;

import java.io.FileWriter;   // Import the FileWriter class
import java.io.IOException;  // Import the IOException class to handle errors

public class GlobalPosSystem {

    Constants constants = new Constants();
    Kinematics kinematics;

    private double[] positionArr = new double[4];
    private HashMap<DcMotorEx, Integer> motorClicksPose = new HashMap<>();
    private HashMap<DcMotorEx, Integer> prevMotorClicks = new HashMap<>();

    HardwareDrive robot;

    public GlobalPosSystem(HardwareDrive robot){
        this.robot = robot;
        for (int i = 0; i < 4; i++){
            positionArr[i] = 0;
        }
        for (DcMotorEx motors : robot.dtMotors){
            motorClicksPose.put(motors, motors.getCurrentPosition()); //(key, value)
            prevMotorClicks.put(motors, motorClicksPose.get(motors)); //(key, value)
        }
    }

    public void grabKinematics(Kinematics k){
        kinematics = k;
    }

    public void calculatePos(){
        for (DcMotorEx motors : robot.dtMotors){
            motorClicksPose.put(motors, motors.getCurrentPosition()); //(key, value)
        }
        if (!goodGap()) return; //this may or may not be useful
        for (DcMotorEx motors : robot.dtMotors){
            prevMotorClicks.put(motors, motorClicksPose.get(motors)); //(key, value)
        }

        //left
        int topL = motorClicksPose.get(robot.dtMotors[0]) - prevMotorClicks.get(robot.dtMotors[0]); //change in top left
        int botL = motorClicksPose.get(robot.dtMotors[1]) - prevMotorClicks.get(robot.dtMotors[1]); //change in bottom left
        double translateL = (topL - botL) / 2.0;
        double rotateL = topL - translateL;
        translateL *= constants.INCHES_PER_CLICK;
        rotateL *= constants.DEGREES_PER_CLICK;

        //right
        int topR = motorClicksPose.get(robot.dtMotors[2]) - prevMotorClicks.get(robot.dtMotors[2]); //change in top right
        int botR = motorClicksPose.get(robot.dtMotors[3]) - prevMotorClicks.get(robot.dtMotors[3]); //change in bottom right
        double translateR = (topR - botR) / 2.0;
        double rotateR = topR - translateR;
        translateR *= constants.INCHES_PER_CLICK;
        rotateR *= constants.DEGREES_PER_CLICK;

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

        try {
            FileWriter myWriter = new FileWriter("gpsLog.txt");
            myWriter.write("GPS Log\n");
            myWriter.write((int) positionArr[0] + "\n");
            myWriter.write((int) positionArr[1] + "\n");
            myWriter.write((int) positionArr[2] + "\n");
            myWriter.write((int) positionArr[3] + "\n\n");
            myWriter.close();
        } catch (IOException e) {
           // System.out.println("An error occurred.");
            e.printStackTrace();
        }
    }

    public void hardResetGPS(){
        //Reset GPS
        for (int i = 0; i < 4; i++){
            positionArr[i] = 0;
        }

        //Reset Motor Clicks
        for (DcMotorEx motors : robot.dtMotors){
            motorClicksPose.put(motors, 0);
        }
    }

    private boolean goodGap(){
        for (int i = 0; i < 3; i++) {
            try{
                if (Math.abs(motorClicksPose.get(robot.dtMotors[i]) - prevMotorClicks.get(robot.dtMotors[i])) <= constants.TOLERANCE) return false;
            } catch (NullPointerException e){
                return false;
            }
        }
        return true;
    }

    public double[] getPositionArr() {
        return positionArr;
    }

    public int[] getMotorClicks(){
        int[] clicks = new int[4];
        for (int i = 0; i < 3; i++){
            clicks[i] = robot.dtMotors[i].getCurrentPosition();
        }
        return clicks;
    }

    public HashMap<DcMotorEx, Integer> getMotorClicksPose () {
        return motorClicksPose;
    }

}