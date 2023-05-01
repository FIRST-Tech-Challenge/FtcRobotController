package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.firstinspires.ftc.teamcode.Old.Robots.BlackoutRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static java.lang.Double.max;
import static java.lang.Double.min;
import static java.lang.Math.abs;
import static java.lang.Math.pow;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

import java.text.DecimalFormat;
import java.util.ArrayList;

@Config
public class RFMotor extends Motor {
    private DcMotorEx rfMotor = null;
    private ArrayList<Double> coefs = null;
    private ArrayList<Double> coefs2 = null;
    private ArrayList<String> inputlogs = new ArrayList<>();
    public static double D = 0.00000, D2 = 0, kP = 4.2E-4, kA = 0.0001*1.2, R = 0,
            MAX_VELOCITY = 15000, MAX_ACCELERATION = 15000, DECEL_DIST = 60, VOLTAGE_CONST=0, RESISTANCE=400;
    private double maxtickcount = 0;
    private double mintickcount = 0;
    private double DEFAULTCOEF1 = 0.0001, DEFAULTCOEF2 = 0.01;
    private double lastError = 0, lastTime = 0;
    private double TICK_BOUNDARY_PADDING = 25, TICK_STOP_PADDING = 25;
    private double power = 0, position = 0, velocity = 0, targetPos = 0, resistance = 0, acceleration = 0, avgResistance, time = op.getRuntime();
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
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        rfMotor.setDirection(direction);
    }

    public void setVelToAnalog(double velToAnalog) {
        kP = velToAnalog;
    }

    public void setPosition(double targetpos) {
        power = 0;
        if (targetpos > maxtickcount) {
            targetpos = maxtickcount;
        }
        if (targetpos < mintickcount) {
            targetpos = mintickcount;
        }
        position = rfMotor.getCurrentPosition();
        targetPos = targetpos;
        acceleration = getVelocity() - velocity;
        velocity += acceleration;
        acceleration /= op.getRuntime() - time;
        time = op.getRuntime();
        getAvgResistance();
        double[] targetMotion = getTargetMotion();
        rfMotor.setPower(kP * (targetMotion[0] - resistance) - kA * targetMotion[1]);
        TelemetryPacket data = new TelemetryPacket();
        data.put("decelDist", getDecelDist());
        data.put("dist", targetPos - position);
        data.put("targetVelocity", targetMotion[0]);
        data.put("velocity", velocity);
        data.put("targetAcceleration", targetMotion[1]);
        data.put("trueAcceleration", acceleration);
        data.put("resistance", resistance);
        BasicRobot.dashboard.sendTelemetryPacket(data);
        lastTime = op.getRuntime();
    }

    public double getResistance() {
        double resistance = 0;
        resistance -= 200 + 0.4 * position - 0.00012 * position * position;
        resistance -= velocity * 0.3 * pow(abs(position) + 1, -.12);
        return resistance;
    }

    public void getAvgResistance() {
        double resistances = 0;
        resistances -= RESISTANCE /* - 0.000135* position * position*/;
//        resistances -= velocity * 0.2 * pow(abs(position) + 1, -.13);
        resistance = resistances* VOLTAGE_CONST;
        resistances -= RESISTANCE/* - 0.000135 * targetPos * targetPos*/;
        avgResistance = resistances / 2 * VOLTAGE_CONST;
    }

    public double getDecelDist() {
        double decelDist = 0;
        if (velocity > 0) {
            decelDist = 0.5 * pow(abs(velocity), 2) / (MAX_ACCELERATION - avgResistance);
        } else {
            decelDist = 0.5 * pow(abs(velocity), 2) / (MAX_ACCELERATION + avgResistance);
        }
        return decelDist;
    }

    public double[] getTargetMotion() {
        double[] targets = {0, 0};
        double DECEL_DIST = getDecelDist(), distance = targetPos - position;
        if (abs(distance) > DECEL_DIST && abs(velocity) < MAX_VELOCITY - 0.1 * MAX_ACCELERATION) {
            if (distance > 0) {
                targets[0] = velocity + .1 * MAX_ACCELERATION * (1 - 1 / (abs(distance - DECEL_DIST) / 100 + 1));
                targets[1] = velocity - targets[0];
            } else {
                targets[0] = velocity - 0.1 * MAX_ACCELERATION * (1 - 1 / (abs(distance - DECEL_DIST) / 100 + 1));
                targets[1] = velocity - targets[0];
            }
        } else if (abs(distance) > DECEL_DIST && abs(distance) > 20) {
            if (distance > 0) {
                targets[0] = MAX_VELOCITY;
                targets[1] = velocity - targets[0];
            } else {
                targets[0] = -MAX_VELOCITY;
                targets[1] = velocity - targets[0];

            }
        } else {
            if (distance < 0) {
                targets[0] = min(-pow((abs(distance)+5) * (MAX_ACCELERATION + avgResistance) * (1 - 1 / (abs(distance) / 200 + 1)), 0.5), 0);
                targets[1] = velocity - targets[0];
            } else {
                targets[0] = max(pow((abs(distance)+5) * (MAX_ACCELERATION - avgResistance) * (1 - 1 / (abs(distance) / 300 + 1)), 0.5), 0);
                targets[1] = velocity - targets[0];

            }
        }
        return targets;
    }

    public boolean atTargetPosition() {
        if (abs(position - targetPos) < TICK_STOP_PADDING) {
            return true;
        } else {
            return false;
        }
    }

    public void setPower(double power) {
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        getAvgResistance();
//        logger.log("/RobotLogs/GeneralRobot", rfMotorName + ",setPower():,Setting Power: " + power, false, false);
        rfMotor.setPower(power/* - kP * resistance*/);
        logger.log("/MotorLogs/RFMotor" + rfMotorName, "Setting Power," + (power - kP * getResistance()), false, false);

    }

    public double getPower() {
        return rfMotor.getPower();
    }

    public double getGRAVITY_CONSTANT() {
        return 0;
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