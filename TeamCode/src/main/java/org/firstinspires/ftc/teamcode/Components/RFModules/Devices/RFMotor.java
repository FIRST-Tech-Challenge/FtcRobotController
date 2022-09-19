package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.firstinspires.ftc.teamcode.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.BasicRobot.op;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;

public class RFMotor extends Motor {
    private final DcMotorEx rfMotor;
    private ArrayList<Double> coefs;
    private double maxtickcount;
    private double mintickcount;
    private final double TICK_BOUNDARY_PADDING = 5;
    private final double TICK_STOP_PADDING = 20;

    /*Initializes the motor
        Inputs:
        motorName: the name of the device | Ex:'motorRightFront'
        motorDirection: the direction of the motor | 0 for Reverse, 1 for Forward | Ex: 0
     */
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

        logger.createFile("RFMotorLog", "Runtime,Action,Value");
    }

    public RFMotor(String motorName, DcMotor.RunMode runMode, boolean resetPos,
                   ArrayList<Double> coefficients, double maxtick, double mintick) {
        rfMotor = (DcMotorEx) op.hardwareMap.dcMotor.get(motorName);
        if (resetPos) {
            rfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        rfMotor.setMode(runMode);
        coefs = coefficients;
        maxtickcount = maxtick;
        mintickcount = mintick;

        logger.createFile("RFMotorLog", "Runtime,Action,Value");
    }

    public RFMotor(String motorName, DcMotor.RunMode runMode, boolean resetPos,
                   double maxtick, double mintick) {
        rfMotor = (DcMotorEx) op.hardwareMap.dcMotor.get(motorName);
        if (resetPos) {
            rfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        rfMotor.setMode(runMode);
        coefs = new ArrayList<Double>();
        coefs.add(1.5);
        coefs.add(150.0);
        maxtickcount = maxtick;
        mintickcount = mintick;

        logger.createFile("RFMotorLog", "Runtime,Action,Value");
    }

    public RFMotor(String motorName, DcMotor.RunMode runMode, boolean resetPos) {
        rfMotor = (DcMotorEx) op.hardwareMap.dcMotor.get(motorName);
        if (resetPos) {
            rfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        rfMotor.setMode(runMode);

        logger.createFile("RFMotorLog", "Runtime,Action,Value");
    }
    //targetpos expected in ticks
    public void setPosition (double targetpos) {
//        if (targetpos >= -1 && targetpos <= 1) {
//            targetpos *= maxtickcount;
//        }
//        else {
//            logger.log("RFMotorLog", "ERROR: RFMotor:: setPosition targetpos expected between -1 -- 1, targetpos = " + targetpos);
//            return;
//        }

        op.telemetry.addData("newPosition", targetpos);
        if(targetpos>maxtickcount){
            targetpos = maxtickcount - TICK_BOUNDARY_PADDING;
        }
        if(targetpos<mintickcount){
            targetpos = mintickcount + TICK_BOUNDARY_PADDING;
        }
        double distance = targetpos-getCurrentPosition();
        op.telemetry.addData("distance", distance);
        while (Math.abs(distance) > TICK_STOP_PADDING) {
            distance = targetpos-getCurrentPosition();
            if (distance > 0) {
                setVelocity(coefs.get(0) * distance + coefs.get(1));
            }
            else if (distance < 0) {
                setVelocity(coefs.get(0) * distance - coefs.get(1));
            }
//            distance/abs(distance) * 4 * (abs(distance) + 100)

            op.telemetry.addData("current position:", getCurrentPosition());

            op.telemetry.addData("distance", distance);
            op.telemetry.update();
        }
        setVelocity(0);
    }

    public void setPower(double power){
        rfMotor.setPower(power);
        logger.log("RFMotorLog", "Setting Power," + power);
    }
    public void setVelocity(double velocity) {
        rfMotor.setVelocity(velocity);
        logger.log("RFMotorLog", "Setting Velocity," + velocity);

    }

    public int getCurrentPosition() {
        logger.log("RFMotorLog", "Current Tick Count," + rfMotor.getCurrentPosition());
        return rfMotor.getCurrentPosition();
    }
    public void setMode(DcMotor.RunMode runMode) {
        rfMotor.setMode(runMode);
        logger.log("RFMotorLog", "Setting Mode," + runMode);
    }

    public double getVelocity() {
        logger.log("RFMotorLog", "Getting Velocity," + rfMotor.getVelocity());
        return rfMotor.getVelocity();
    }
}