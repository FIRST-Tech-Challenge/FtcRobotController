package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.firstinspires.ftc.teamcode.Old.Robots.BlackoutRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static java.lang.Math.abs;
import static java.lang.Math.pow;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.text.DecimalFormat;
import java.util.ArrayList;

@Config
public class RFMotor extends Motor {
    private DcMotorEx rfMotor = null;
    private ArrayList<Double> coefs = null;
    private ArrayList<Double> coefs2 = null;
    private ArrayList<String> inputlogs = new ArrayList<>();
    public static double D = 0.00000, D2 = 0, minVelocity = -5000, VEL_TO_ANALOG = .0014, kA = 0;
    private double maxtickcount = 0;
    private double mintickcount = 0;
    private double DEFAULTCOEF1 = 0.0001, DEFAULTCOEF2 = 0.01;
    private double GRAVITY_CONSTANT = 0.1;
    private double lastError = 0, lastTime = 0;
    private double TICK_BOUNDARY_PADDING = 10, TICK_STOP_PADDING = 10;
    private double power = 0;
    private String rfMotorName;

    private static final DecimalFormat df = new DecimalFormat("0.00");

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

        logger.createFile("/MotorLogs/RFMotor" + motorName, "Runtime    Component               " +
                "Function               Action");
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

        logger.createFile("/MotorLogs/RFMotor" + motorName, "Runtime    Component               " +
                "Function               Action");
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
        coefs = new ArrayList<>();
        coefs.add(DEFAULTCOEF1);
        coefs.add(DEFAULTCOEF2);
        maxtickcount = maxtick;
        mintickcount = mintick;

        logger.createFile("/MotorLogs/RFMotor" + rfMotorName, "Runtime    Component               " +
                "Function               Action");
    }

    //for chassis wheels where you only need it to spin continuously
    public RFMotor(String motorName, DcMotor.RunMode runMode, boolean resetPos) {
        rfMotor = (DcMotorEx) op.hardwareMap.dcMotor.get(motorName);
        rfMotorName = motorName;
        if (resetPos) {
            rfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        rfMotor.setMode(runMode);

        logger.createFile("/MotorLogs/RFMotor" + motorName, "Runtime    Component               " +
                "Function               Action");
    }

    public void setPosition(double targetpos) {
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        power = 0;
//        if (targetpos >= -1 && targetpos <= 1) {
//            targetpos *= maxtickcount;
//        }
//        else {
//            logger.log("RFMotorLog", "ERROR: RFMotor:: setPosition targetpos expected between -1 -- 1, targetpos = " + targetpos);
//            return;
//        }

//        op.telemetry.addData("newPosition", targetpos);
        if (targetpos > maxtickcount) {
            targetpos = maxtickcount - TICK_BOUNDARY_PADDING;
        }
        if (targetpos < mintickcount) {
            targetpos = mintickcount + TICK_BOUNDARY_PADDING;
        }
        double distance = targetpos - getCurrentPosition();

        double targetVelocity = 0, targetAccelVelocity = 0;
        if (distance > 0) {
            for (int i = 0; i < coefs.size(); i++) {
                if (i != coefs.size() - 1 || abs(distance) > TICK_STOP_PADDING) {
                    if (coefs.size() - i - 1 == 2) {
                        targetVelocity += pow(distance, 0.6) * coefs.get(i);
                    } else {
                        targetVelocity += pow(distance, coefs.size() - i - 1) * coefs.get(i);
                    }
                }
            }
            for (int i = 0; i < coefs.size(); i++) {
                if (i != coefs.size() - 1 || abs(distance) > TICK_STOP_PADDING) {
                    targetAccelVelocity += pow(distance - targetVelocity / 10, coefs.size() - i - 1) * coefs.get(i);
                }
            }
        } else if (distance < 0) {
            for (int i = 0; i < coefs.size(); i++) {
                if (i != coefs.size() - 1 || abs(distance) > TICK_STOP_PADDING) {
                    if (coefs.size() - i - 1 == 2) {
                        targetVelocity -= pow(abs(distance), 0.6) * coefs.get(i);
                    } else {
                        targetVelocity -= abs(pow(distance, coefs.size() - i - 1)) * coefs.get(i);
                    }
                }
            }
            for (int i = 0; i < coefs.size(); i++) {
                if (i != coefs.size() - 1 || abs(distance) > TICK_STOP_PADDING) {
                    targetAccelVelocity -= pow(distance - targetVelocity / 10, coefs.size() - i - 1) * coefs.get(i);
                }
            }
        }

        if (targetVelocity < minVelocity) {
            targetVelocity = minVelocity;
        }
        double currentVelocity = rfMotor.getVelocity();
        double error = targetVelocity - currentVelocity;
        double dStuff = 0;
        if (abs(error) < abs(lastError)) {
            dStuff = (error - lastError) / (op.getRuntime() - lastTime) * D2;
        } else {
            dStuff = (error - lastError) / (op.getRuntime() - lastTime) * D2 / 2;
        }
        double accelPow = (targetAccelVelocity - targetVelocity) * kA;
//        if(targetVelocity<0){
//            VEL_TO_ANALOG=0.001;
//            D=0;
//        }else{
//            VEL_TO_ANALOG=0.0022;
//            D=0.00000507;
//        }
        power = dStuff + error * VEL_TO_ANALOG;
        rfMotor.setPower(dStuff + error * D + targetVelocity * VEL_TO_ANALOG + GRAVITY_CONSTANT + accelPow);
        lastError = error;
        lastTime = op.getRuntime();
    }

    public void setPower(double power) {
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        logger.log("/MotorLogs/RFMotor" + rfMotorName, "Setting Power," + power, false, false);
//        logger.log("/RobotLogs/GeneralRobot", rfMotorName + ",setPower():,Setting Power: " + power, false, false);
        rfMotor.setPower(power);
    }

    public double getPower() {
        return rfMotor.getPower();
    }

    public double getGRAVITY_CONSTANT() {
        return GRAVITY_CONSTANT;
    }

    public void setVelocity(double velocity) {
        if (rfMotor.getVelocity() != velocity) {
//            inputlogs.add(rfMotorName);
//            inputlogs.add("setVelocity()");
//            inputlogs.add("Setting Velocity: " + velocity);
//            logger.log("/MotorLogs/RFMotor", rfMotorName + ",setVelocity()," +
//                    "Setting Velocity: " + df.format(velocity), true, true);
//            inputlogs.clear();
//            logger.log("/MotorLogs/RFMotor" + rfMotorName, "Setting Velocity," + velocity);
//            logger.log("/RobotLogs/GeneralRobot", rfMotorName + "\nsetVelocity():\nSetting Velocity:" + velocity);
        }
        rfMotor.setVelocity(velocity);
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
//            inputlogs.add(rfMotorName);
//            inputlogs.add("setMode()");
//            inputlogs.add("Setting RunMode: " + runMode);
            logger.log("/MotorLogs/RFMotor", rfMotorName + ",setMode(),Setting RunMode: " + runMode,
                    true, true);
//            inputlogs.clear();

//            logger.log("/MotorLogs/RFMotor" + rfMotorName, "Setting Mode," + runMode);
//            logger.log("/RobotLogs/GeneralRobot", rfMotorName + "setMode():\nSetting Mode," + runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        rfMotor.setZeroPowerBehavior(behavior);
    }

    public double getVelocity() {
        return rfMotor.getVelocity();
    }

    public double getTICK_BOUNDARY_PADDING() {
        return TICK_BOUNDARY_PADDING;
    }

    public double getTICK_STOP_PADDING() {
        return TICK_STOP_PADDING;
    }

    public void setTICK_BOUNDARY_PADDING(double p_TICK_BOUNDARY_PADDING) {
        TICK_BOUNDARY_PADDING = p_TICK_BOUNDARY_PADDING;
    }

    public void setTICK_STOP_PADDING(double p_TICK_STOP_PADDING) {
        TICK_STOP_PADDING = p_TICK_STOP_PADDING;
    }
}