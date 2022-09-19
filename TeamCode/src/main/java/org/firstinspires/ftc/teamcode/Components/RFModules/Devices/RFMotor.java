package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Old.Robots.BlackoutRobot.logger;

import static java.lang.Math.abs;
import static java.lang.Math.pow;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.io.File;
import java.util.ArrayList;

public class RFMotor extends Motor {
    private DcMotorEx rfMotor = null;
    private ArrayList<Double> coefs = null;
    private ArrayList<String> inputlogs = new ArrayList<>();

    private double maxtickcount = 0;
    private double mintickcount = 0;
    private double DEFAULTCOEF1 = 1.5, DEFAULTCOEF2 = 150.0;
    private double TICK_BOUNDARY_PADDING = 5, TICK_STOP_PADDING=5;
    private double velocity = 0;
    private String rfMotorName;

    /*Initializes the motor
        Inputs:
        motorName: the name of the device | Ex:'motorRightFront'
        motorDirection: the direction of the motor | 0 for Reverse, 1 for Forward | Ex: 0
     */

    //for motors used for complex functions

    public RFMotor(String motorName, DcMotorSimple.Direction motorDirection, DcMotor.RunMode runMode,
                   boolean resetPos, ArrayList<Double> coefficients,
                   double maxtick, double mintick) {
        rfMotor = (DcMotorEx) op.hardwareMap.dcMotor.get(motorName);
        rfMotor.setDirection(motorDirection);
        if (resetPos) {
            rfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        rfMotor.setMode(runMode);
        coefs = coefficients;
        maxtickcount = maxtick;
        mintickcount = mintick;

        logger.createFile("/MotorLogs/RFMotor" + motorName, "Runtime,Action,Value");
    }

    //same as above but assuming motor direction is foward
    public RFMotor(String motorName, DcMotor.RunMode runMode, boolean resetPos,
                   ArrayList<Double> coefficients, double maxtick, double mintick) {
        rfMotor = (DcMotorEx) op.hardwareMap.dcMotor.get(motorName);
        rfMotorName = motorName;
        if (resetPos) {
            rfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        rfMotor.setMode(runMode);
        coefs = coefficients;
        maxtickcount = maxtick;
        mintickcount = mintick;

        logger.createFile("/MotorLogs/RFMotor" + motorName, "Runtime,Action,Value");
    }

    //same as above but using default coefficients
    public RFMotor(String motorName, DcMotor.RunMode runMode, boolean resetPos,
                   double maxtick, double mintick) {
        rfMotor = (DcMotorEx) op.hardwareMap.dcMotor.get(motorName);
        rfMotorName = motorName;
        if (resetPos) {
            rfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        rfMotor.setMode(runMode);
        coefs = new ArrayList<Double>();
        coefs.add(DEFAULTCOEF1);
        coefs.add(DEFAULTCOEF2);
        maxtickcount = maxtick;
        mintickcount = mintick;

        logger.createFile("/MotorLogs/RFMotor" + motorName, "Runtime,Action,Value");
    }

    //for chassis wheels where you only need it to spin continuously
    public RFMotor(String motorName, DcMotor.RunMode runMode, boolean resetPos) {
        rfMotor = (DcMotorEx) op.hardwareMap.dcMotor.get(motorName);
        rfMotorName = motorName;
        if (resetPos) {
            rfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        rfMotor.setMode(runMode);

        logger.createFile("/MotorLogs/RFMotor" + motorName, "Runtime,Action,Value");
    }

    //BUG WITH CALCULATION
    public void setPosition (double targetpos) {
        velocity = 0;
//        if (targetpos >= -1 && targetpos <= 1) {
//            targetpos *= maxtickcount;
//        }
//        else {
//            logger.log("RFMotorLog", "ERROR: RFMotor:: setPosition targetpos expected between -1 -- 1, targetpos = " + targetpos);
//            return;
//        }

//        op.telemetry.addData("newPosition", targetpos);
        if(targetpos>maxtickcount){
            targetpos = maxtickcount - TICK_BOUNDARY_PADDING;
        }
        if(targetpos<mintickcount){
            targetpos = mintickcount + TICK_BOUNDARY_PADDING;
        }
        double distance = targetpos-getCurrentPosition();

        if (Math.abs(distance) > TICK_STOP_PADDING) {
            distance = targetpos-getCurrentPosition();
            if (distance > 0) {
                for (int i = 0; i < coefs.size(); i++){
                    velocity += pow(distance, coefs.size() - i - 1) * coefs.get(i);
                }
                setVelocity(velocity);
            }
            else if (distance < 0) {
                for (int i = 0; i < coefs.size(); i++) {
                    velocity -= pow(Math.abs(distance), coefs.size() - i - 1) * coefs.get(i);
                }
                setVelocity(velocity);
            }
            else {
                logger.logMessage("/MotorLogs/RFMotor" + rfMotorName, "ERROR: distance should not be equal to 0 in setPosition() in RFMotor");
            }
        }
        else {
            setVelocity(0);
        }
    }

    public void setPower(double power){
        rfMotor.setPower(power);
        if (rfMotor.getPower() != power) {
            inputlogs.add(rfMotorName);
            inputlogs.add("setPower()");
            inputlogs.add("Setting Power: " + power);
            logger.logRegulated("/RobotLogs/GeneralRobot", inputlogs);
            inputlogs.clear();
//            logger.log("/MotorLogs/RFMotor" + rfMotorName, "Setting Power," + power);
//            logger.log("/RobotLogs/GeneralRobotLog", rfMotorName + "\nsetPower():\nSetting Power:" + power);
        }
    }
    public void setVelocity(double velocity) {
        rfMotor.setVelocity(velocity);
        if (rfMotor.getVelocity() != velocity) {
            inputlogs.add(rfMotorName);
            inputlogs.add("setVelocity()");
            inputlogs.add("Setting Velocity: " + velocity);
            logger.logRegulated("/RobotLogs/GeneralRobot", inputlogs);
            inputlogs.clear();
//            logger.log("/MotorLogs/RFMotor" + rfMotorName, "Setting Velocity," + velocity);
//            logger.log("/RobotLogs/GeneralRobot", rfMotorName + "\nsetVelocity():\nSetting Velocity:" + velocity);
        }
    }

    public int getCurrentPosition() {
//        inputlogs.add(rfMotorName);
//        inputlogs.add("getCurrentPosition()");
//        inputlogs.add("Getting Position: " + rfMotor.getCurrentPosition());
//        inputlogs.clear();

//        logger.log("/RobotLogs/GeneralRobot", inputlogs);
//        logger.log("/MotorLogs/RFMotor" + rfMotorName, "Current Tick Count," + rfMotor.getCurrentPosition());
        return rfMotor.getCurrentPosition();
    }
    public void setMode(DcMotor.RunMode runMode) {
        rfMotor.setMode(runMode);
        if (rfMotor.getMode() != runMode) {
            inputlogs.add(rfMotorName);
            inputlogs.add("setMode()");
            inputlogs.add("Setting RunMode: " + runMode);
            logger.log("/RobotLogs/GeneralRobot", inputlogs);
            inputlogs.clear();

//            logger.log("/MotorLogs/RFMotor" + rfMotorName, "Setting Mode," + runMode);
//            logger.log("/RobotLogs/GeneralRobot", rfMotorName + "setMode():\nSetting Mode," + runMode);
        }
    }

    public double getVelocity() {
        return rfMotor.getVelocity();
    }
}