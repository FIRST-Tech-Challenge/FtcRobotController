package org.firstinspires.ftc.teamcode.Localization;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Utils.Utils;

public class DeadWheelOdometery {
    public HardwareMap hwMap;
    public DcMotorEx leftEncoder;
    public DcMotorEx centerEncoder;
    public DcMotorEx rightEncoder;
    public static double ticksPerRotation = 2000;
    public static double radius = 24; //TODO: convert to inches
    public double currentLeftVal = 0; //[ticks]
    public double currentCenterVal = 0; //[ticks]
    public double currentRightVal = 0; //[ticks]
    public double lastLeftVal = 0; //[ticks]
    public double lastCenterVal = 0; //[ticks]
    public double lastRightVal = 0; //[ticks]
    public static double trackWidth = 341.15; //TODO: convert to inches
    public static double forwardOffset = 200; //TODO: convert to inches
    public ElapsedTime dt = new ElapsedTime(); //[s]

    public void initialize(HardwareMap hardwareMap, DcMotorEx motor0, DcMotorEx motor1, DcMotorEx motor3) {
        hwMap = hardwareMap;
        leftEncoder = motor0;
        centerEncoder = motor1;
        rightEncoder = motor3;
        dt.reset();
    }
    public void updateEncoderValues(){
        currentLeftVal = leftEncoder.getCurrentPosition();
        currentCenterVal = centerEncoder.getCurrentPosition();
        currentRightVal = rightEncoder.getCurrentPosition();
    }
    public void pushBackValues(){
        lastLeftVal=currentLeftVal;
        lastCenterVal=currentCenterVal;
        lastRightVal=currentRightVal;
    }
    public double convertToDistance(double currentVal, double lastVal){
        return 2*Math.PI*radius*(currentVal-lastVal)/ticksPerRotation;
    }
    public SimpleMatrix calculate(double theta){
        updateEncoderValues();
        double relativeChangeX = (convertToDistance(currentLeftVal, lastLeftVal) + convertToDistance(currentRightVal, lastRightVal))/2;
        double changeInHeading = (convertToDistance(currentRightVal, lastRightVal) - convertToDistance(currentLeftVal, lastLeftVal))/trackWidth;
        double relativeChangeY = convertToDistance(currentCenterVal, lastCenterVal)-forwardOffset*changeInHeading;
        SimpleMatrix deltaPoseBody = new SimpleMatrix(
                new double[][]{
                        new double[]{relativeChangeX},
                        new double[]{relativeChangeY},
                        new double[]{changeInHeading}
                }
        );
        SimpleMatrix poseExponentials = new SimpleMatrix(
                new double[][]{
                        new double[]{Math.sin(changeInHeading) / changeInHeading, (Math.cos(changeInHeading) - 1) / changeInHeading, 0},
                        new double[]{(Math.cos(changeInHeading) - 1) / changeInHeading, Math.sin(changeInHeading) / changeInHeading, 0},
                        new double[]{0, 0, 1}
                }
        );

        if (changeInHeading == 0) {
            poseExponentials = SimpleMatrix.identity(3);
        }
        SimpleMatrix deltaPose = Utils.rotateBodyToGlobal(poseExponentials, theta).mult(deltaPoseBody);
        pushBackValues();
        return deltaPose;
    }
}
