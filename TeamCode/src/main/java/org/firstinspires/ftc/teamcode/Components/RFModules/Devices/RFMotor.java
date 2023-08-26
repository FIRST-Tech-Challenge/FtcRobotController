package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static java.lang.Double.max;
import static java.lang.Double.min;
import static java.lang.Math.abs;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RFMotorPoseSim;
import org.firstinspires.ftc.teamcode.roadrunner.util.DashboardUtil;

import java.util.ArrayList;
import java.util.Arrays;

@Config
public class RFMotor extends Motor {
    private final DcMotorEx rfMotor;
    private ArrayList<Double> coefs = null;
    public static double kP = 0.009;
    public static final double kD = 0.00001;
    public static final double kV = 0.0003;
    public static final double kA = 0.00004;
    public static final double kS = 0.15;
    public static final double MAX_ACCELERATION_UP = 6000;
    public static final double MAX_ACCELERATION_DOWN = 12000;
    public static final double RESISTANCE = 400;
    public static final double gravity = 0.2;
    private final double MAX_VELOCITY_UP = 1475 - 225 * (13.5 - BasicRobot.voltageSensor.getVoltage());
    private final double MAX_VELOCITY_DOWN = 3500;
    private double relativeDist, direction, peakVelo, J, decelDist;
    private final double[][] calculatedIntervals = new double[4][8];
    private final double[][][] calculatedMotions = new double[3][7][5];
    private double[] timeIntervals = new double[8];
    private double[] distances = new double[8];
    private double[] velocities = new double[8];
    private double[] positions = new double[8];
    private double maxtickcount = 0;
    private double mintickcount = 0;
    private final double DEFAULTCOEF1 = 0.0001;
    private final double DEFAULTCOEF2 = 0.01;
    private final double lastError = 0;
    private double lastTime = 0;
    private double additionalTicks = 0;
    private double TICK_BOUNDARY_PADDING = 10, TICK_STOP_PADDING = 20;

    private double currentAcceleration, currentPos, currentTickPos;
    private double power = 0, position = 0, velocity = 0, targetPos = 0, resistance = 0, acceleration = 0, avgResistance;
    private String rfMotorName;
    private boolean isSim = false, sameTarget = false;

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
        additionalTicks = 0;
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
        additionalTicks = 0;
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

    public void setPosition(double p_targetPos, double curve) {
        if (targetPos == p_targetPos) {
            sameTarget = true;
        } else {
            targetPos = p_targetPos;
            sameTarget = false;
        }
        power = 0;
        if (p_targetPos > maxtickcount) {
            p_targetPos = maxtickcount;
        }
        if (p_targetPos < mintickcount) {
            p_targetPos = mintickcount;
        }
        position = getCurrentPosition();
        targetPos = p_targetPos;
        acceleration = getVelocity() - velocity;
        velocity += acceleration;
        acceleration /= (time - lastTime);
        getAvgResistance();

        double[] targetMotion = getTargetMotion(curve);

        double power = (kV * targetMotion[0] + kA * targetMotion[1] +
                kP * (getTargetPosition(BasicRobot.time) - position) + kD * (getTargetVelocity(BasicRobot.time) - velocity) + gravity);
        power = abs(power) / power * min(1, abs(power));

        if (abs(targetPos - position) > TICK_BOUNDARY_PADDING && abs(velocity) < 3) {
            if (power < 0) {
                power -= kS;
            } else {
                power += kS;
            }
        }
        setRawPower(power);
        lastTime = time;
    }


    public double[] getTargetMotion(double curve) {
        double[] targets = {0, 0};

        relativeDist = targetPos - getCurrentPosition();

        if (relativeDist == 0) {
            return targets;
        }

        if (sameTarget) {
            targets[0] = getTargetVelocity(BasicRobot.time);
            targets[1] = getTargetAcceleration(BasicRobot.time);
            return targets;
        }

        direction = abs(relativeDist) / relativeDist;

        if (isSim) {
            velocities[0] = direction * velocity;
            positions[0] = direction * currentTickPos;
        } else {
            velocities[0] = direction * rfMotor.getVelocity();
            positions[0] = direction * rfMotor.getCurrentPosition();
        }

        if (velocities[0] == 0) {
            peakVelo = min((131 - 38 * curve) / 131 * sqrt(getMaxAcceleration() * abs(relativeDist)), getMaxVelocity());
        } else {
            peakVelo = min((131 - 38 * curve) / 131 * sqrt(getMaxAcceleration() * (abs(relativeDist) -
                    abs(velocities[0]) / velocities[0] * pow(velocities[0], 2) / (2 * getMaxAcceleration()))), getMaxVelocity());
        }

        J = getMaxAcceleration() / ((peakVelo / (getMaxAcceleration() * (1 - curve / 2))) * curve / 2);

        velocities[0] = min(velocities[0], peakVelo);

        calculateIntervals();
        calculateMotions();
        timeIntervals = calculatedIntervals[0];
        distances = calculatedIntervals[1];
        velocities = calculatedIntervals[2];
        positions = calculatedIntervals[3];
        decelDist = calculatedIntervals[3][7] - calculatedIntervals[3][4];

        if (isSim) {
            velocities[0] = velocity;
            positions[0] = currentTickPos;
            isSim = false;
        } else {
            velocities[0] = rfMotor.getVelocity();
            positions[0] = rfMotor.getCurrentPosition();
        }

        targets[0] = getTargetVelocity(BasicRobot.time);
        targets[1] = getTargetAcceleration(BasicRobot.time);

        decelDist = getTargetPosition(timeIntervals[7]) - getTargetPosition(timeIntervals[4]);

        return targets;
    }

    public void calculateIntervals() {
        double[] temp_timeIntervals = new double[8];
        double cruiseTime;
        double[] temp_distances = new double[8];
        double semiTotal;
        double cruiseAccelTime;
        double[] temp_velocities = new double[8];
        double[] temp_positions = new double[8];

        temp_timeIntervals[0] = BasicRobot.time;
        temp_timeIntervals[2] = max(sqrt((peakVelo - velocities[0]) / J), (max(0, (peakVelo - velocities[0])
                / getMaxAcceleration())));
        temp_timeIntervals[1] = min(temp_timeIntervals[2], getMaxAcceleration() / J);
        temp_timeIntervals[3] = temp_timeIntervals[1] + temp_timeIntervals[2];

        cruiseAccelTime = temp_timeIntervals[2] - temp_timeIntervals[1];

        temp_velocities[0] = velocities[0];
        temp_velocities[1] = temp_velocities[0] + J * pow(temp_timeIntervals[1], 2) / 2;
        temp_velocities[2] = temp_velocities[1] + getMaxAcceleration() * cruiseAccelTime;
        temp_velocities[3] = peakVelo;
        temp_velocities[4] = peakVelo;
        temp_velocities[5] = temp_velocities[4] - J * pow(getMaxAcceleration() / J, 2) / 2;
        temp_velocities[6] = J * pow(getMaxAcceleration() / J, 2) / 2;
        temp_velocities[7] = 0;

        temp_distances[0] = 0;
        temp_distances[1] = J * pow(temp_timeIntervals[1], 3) / 6 + temp_velocities[0] * temp_timeIntervals[1];
        temp_distances[2] = temp_velocities[1] * cruiseAccelTime + getMaxAcceleration() * pow(cruiseAccelTime, 2) / 2;
        temp_distances[3] = temp_velocities[2] * temp_timeIntervals[1] + J * pow(temp_timeIntervals[1], 3) / 6;
        temp_distances[5] = temp_velocities[4] * getMaxAcceleration() / J - J * pow(getMaxAcceleration() / J, 3) / 6;
        temp_distances[6] = temp_velocities[5] * (peakVelo / getMaxAcceleration() - getMaxAcceleration() / J) -
                getMaxAcceleration() * pow(peakVelo / getMaxAcceleration() - getMaxAcceleration() / J, 2) / 2;
        temp_distances[7] = J * pow(getMaxAcceleration() / J, 3) / 6;

        semiTotal = temp_distances[1] + temp_distances[2] + temp_distances[3] + temp_distances[5] +
                temp_distances[6] + temp_distances[7];

        cruiseTime = (abs(relativeDist) - semiTotal) / peakVelo;

        temp_timeIntervals[4] = temp_timeIntervals[3] + cruiseTime;
        temp_timeIntervals[5] = temp_timeIntervals[4] + getMaxAcceleration() / J;
        temp_timeIntervals[6] = temp_timeIntervals[4] + peakVelo / getMaxAcceleration();
        temp_timeIntervals[7] = temp_timeIntervals[6] + getMaxAcceleration() / J;

        temp_distances[4] = temp_velocities[3] * cruiseTime;

        temp_positions[1] = temp_positions[0] + temp_distances[1];
        temp_positions[2] = temp_positions[1] + temp_distances[2];
        temp_positions[3] = temp_positions[2] + temp_distances[3];
        temp_positions[4] = temp_positions[3] + temp_distances[4];
        temp_positions[5] = temp_positions[4] + temp_distances[5];
        temp_positions[6] = temp_positions[5] + temp_distances[6];
        temp_positions[7] = temp_positions[6] + temp_distances[7];

        calculatedIntervals[0] = temp_timeIntervals;
        calculatedIntervals[1] = temp_distances;
        calculatedIntervals[2] = temp_velocities;
        calculatedIntervals[3] = temp_positions;

    }

    public void calculateMotions() {
        double[][] acceleration = new double[7][5];
        double[][] velocity = new double[7][5];
        double[][] position = new double[7][5];

        for (double[] type : acceleration) {
            Arrays.fill(type, -1);
        }

        for (double[] type : velocity) {
            Arrays.fill(type, -1);
        }

        for (double[] type : position) {
            Arrays.fill(type, -1);
        }

        acceleration[0][1] = J;

        acceleration[1][0] = J * calculatedIntervals[0][1];

        acceleration[2][1] = -J;
        acceleration[2][4] = calculatedIntervals[0][3];

        acceleration[4][1] = -J;
        acceleration[4][4] = calculatedIntervals[0][4];

        acceleration[5][0] = -getMaxAcceleration();

        acceleration[6][1] = J;
        acceleration[6][4] = calculatedIntervals[0][7];

        velocity[0][0] = calculatedIntervals[2][0];
        velocity[0][2] = J / 2;

        velocity[1][0] = calculatedIntervals[2][1];
        velocity[1][1] = getMaxAcceleration();
        velocity[1][4] = calculatedIntervals[0][1];

        velocity[2][0] = calculatedIntervals[2][2] + J / 2 * pow(calculatedIntervals[0][1], 2);
        velocity[2][2] = -J / 2;
        velocity[2][4] = calculatedIntervals[0][3];

        velocity[3][0] = calculatedIntervals[2][3];

        velocity[4][0] = calculatedIntervals[2][4];
        velocity[4][2] = -J / 2;
        velocity[4][4] = calculatedIntervals[0][4];

        velocity[5][0] = calculatedIntervals[2][5];
        velocity[5][1] = -getMaxAcceleration();
        velocity[5][4] = calculatedIntervals[0][5];

        velocity[6][2] = J / 2;
        velocity[6][4] = calculatedIntervals[0][7];


        position[0][0] = calculatedIntervals[3][0];
        position[0][1] = calculatedIntervals[2][0];
        position[0][3] = J / 6;

        position[1][0] = calculatedIntervals[3][1];
        position[1][1] = calculatedIntervals[2][1];
        position[1][2] = getMaxAcceleration() / 2;
        position[1][4] = calculatedIntervals[0][1];

        position[2][0] = calculatedIntervals[3][2];
        position[2][1] = calculatedIntervals[2][2];
        position[2][3] = J / 6;
        position[2][4] = calculatedIntervals[0][2];

        position[3][0] = calculatedIntervals[3][3];
        position[3][1] = calculatedIntervals[2][3];
        position[3][4] = calculatedIntervals[0][3];

        position[4][0] = calculatedIntervals[3][4];
        position[4][1] = calculatedIntervals[2][4];
        position[4][3] = -J / 6;
        position[4][4] = calculatedIntervals[0][4];

        position[5][0] = calculatedIntervals[3][5];
        position[5][1] = calculatedIntervals[2][5];
        position[5][2] = -getMaxAcceleration() / 2;
        position[5][4] = calculatedIntervals[0][5];

        position[6][0] = calculatedIntervals[3][7];
        position[6][3] = J / 6;
        position[6][4] = calculatedIntervals[0][7];

        calculatedMotions[0] = acceleration;
        calculatedMotions[1] = velocity;
        calculatedMotions[2] = position;

    }

    public double getTargetPosition(double p_time) {
        p_time -= timeIntervals[0];
        if (p_time >= timeIntervals[7]) {
            return positions[7] * direction + positions[0];
        }
        double currentTargetPos = 0;
        double firstInterval;
        for (int i = 0; i < 7; i++) {
            if (i == 0) {
                firstInterval = 0;
            } else {
                firstInterval = timeIntervals[i];
            }
            if (p_time >= firstInterval && p_time < timeIntervals[i + 1]) {
                for (int j = 0; j < 4; j++) {
                    if (calculatedMotions[2][i][4] != -1) {
                        if (calculatedMotions[2][i][j] != -1) {
                            currentTargetPos += pow(p_time - calculatedMotions[2][i][4], j) * calculatedMotions[2][i][j];
                        }
                    } else {
                        if (calculatedMotions[2][i][j] != -1) {
                            currentTargetPos += pow(p_time, j) * calculatedMotions[2][i][j];
                        }
                    }
                }
                break;
            }
        }

        return direction * currentTargetPos + positions[0];
    }

    public double getTargetVelocity(double p_time) {
        p_time -= timeIntervals[0];
        if (p_time >= timeIntervals[7]) {
            return velocities[7] * direction;
        }
        double currentTargetVelo = 0;
        double firstInterval;
        for (int i = 0; i < 7; i++) {
            if (i == 0) {
                firstInterval = 0;
            } else {
                firstInterval = timeIntervals[i];
            }
            if (p_time >= firstInterval && p_time < timeIntervals[i + 1]) {
                for (int j = 0; j < 4; j++) {
                    if (calculatedMotions[1][i][4] != -1) {
                        if (calculatedMotions[1][i][j] != -1) {
                            currentTargetVelo += pow(p_time - calculatedMotions[1][i][4], j) * calculatedMotions[1][i][j];
                        }
                    } else {
                        if (calculatedMotions[1][i][j] != -1) {
                            currentTargetVelo += pow(p_time, j) * calculatedMotions[1][i][j];
                        }
                    }
                }
                break;
            }
        }
        return direction * currentTargetVelo;
    }

    public double getTargetAcceleration(double p_time) {
        p_time -= timeIntervals[0];
        if (p_time > timeIntervals[7]) {
            return 0;
        }
        double currentTargetAccel = 0;
        double firstInterval;
        for (int i = 0; i < 7; i++) {
            if (i == 0) {
                firstInterval = 0;
            } else {
                firstInterval = timeIntervals[i];
            }
            if (p_time > firstInterval && p_time < timeIntervals[i + 1]) {
                for (int j = 0; j < 4; j++) {
                    if (calculatedMotions[0][i][4] != -1) {
                        if (calculatedMotions[0][i][j] != -1) {
                            currentTargetAccel += pow(p_time - calculatedMotions[0][i][4], j) * calculatedMotions[0][i][j];
                        }
                    } else {
                        if (calculatedMotions[0][i][j] != -1) {
                            currentTargetAccel += pow(p_time, j) * calculatedMotions[0][i][j];
                        }
                    }
                }
                break;
            }
        }
        return direction * currentTargetAccel;
    }

    public double getMaxVelocity() {
        if (direction == 1) {
            return MAX_VELOCITY_UP;
        } else {
            return MAX_VELOCITY_DOWN;
        }
    }

    public double getMaxAcceleration() {
        if (direction == 1) {
            return MAX_ACCELERATION_UP;
        } else {
            return MAX_ACCELERATION_DOWN;
        }
    }

    public double getTarget() {
        return targetPos;
    }

    public double getResistance() {
//        double resistance = 0;
//        resistance -= 200 + 0.4 * position - 0.00012 * position * position;
//        resistance -= velocity * 0.3 * pow(abs(position) + 1, -.12);
        return -RESISTANCE;
    }

    public void getAvgResistance() {
//        double resistances = 0;
//        resistances -= RESISTANCE /* - 0.000135* position * position*/;
////        resistances -= velocity * 0.2 * pow(abs(position) + 1, -.13);
////        resistance = resistances* VOLTAGE_CONST;
//        resistances -= RESISTANCE/* - 0.000135 * targetPos * targetPos*/;
        resistance = -RESISTANCE;
        avgResistance = -RESISTANCE;
    }

    public boolean atTargetPosition() {
        return abs(position - targetPos) < TICK_STOP_PADDING;
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        rfMotor.setDirection(direction);
    }

    public void setVelToAnalog(double velToAnalog) {
        kP = velToAnalog;
    }

    public void setCurrentPosition(double position) {
        additionalTicks = position - rfMotor.getCurrentPosition();
    }

    public void setPower(double power) {
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        getAvgResistance();
//        logger.log("/RobotLogs/GeneralRobot", rfMotorName + ",setPower():,Setting Power: " + power, false, false);
        rfMotor.setPower(power - kP * resistance);
//        logger.log("/MotorLogs/RFMotor" + rfMotorName, "Setting Power," + (power - kP * getResistance()), false, false);

    }

    public void setRawPower(double power) {
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        getAvgResistance();
//        logger.log("/RobotLogs/GeneralRobot", rfMotorName + ",setPower():,Setting Power: " + power, false, false);
        rfMotor.setPower(power);
//        logger.log("/MotorLogs/RFMotor" + rfMotorName, "Setting Power," + (power), false, false);

    }

    public double getPower() {
        return rfMotor.getPower();
    }

    public double getGRAVITY_CONSTANT() {
        return getResistance();
    }

    public void setVelocity(double velocity) {
        rfMotor.setVelocity(velocity);
    }

    public int getCurrentPosition() {
        return rfMotor.getCurrentPosition() + (int) additionalTicks;
    }

    public void setMode(DcMotor.RunMode runMode) {
        rfMotor.setMode(runMode);
        if (rfMotor.getMode() != runMode) {
            logger.log("/MotorLogs/RFMotor", rfMotorName + ",setMode(),Setting RunMode: " + runMode,
                    true, true);
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

    double lastUpdateTime = 0.0;

    public void getTargets(double p_targetPos, double p_tickPos, double p_targetVelo, double p_targetAccel) {
        currentPos = p_targetPos;
        currentTickPos = p_tickPos;
        velocity = p_targetVelo;
        currentAcceleration = p_targetAccel;
        lastUpdateTime = time;
    }

    public void updateSim() {
        Pose2d targetPose = new Pose2d(currentPos, 0, 0);
        lastUpdateTime = time;
        Canvas fieldOverlay = packet.fieldOverlay();
        fieldOverlay.setStrokeWidth(1);
        fieldOverlay.setStroke("#4CAF50");
        DashboardUtil.drawRobot(fieldOverlay, targetPose);
    }

    public void update() {
        getTargets(getTargetPosition(BasicRobot.time) * 0.05, getTargetPosition(BasicRobot.time),
                getTargetVelocity(BasicRobot.time), getTargetAcceleration(BasicRobot.time));
        updateSim();
    }

    public void setIsSim(boolean p_IsSim) {
        isSim = p_IsSim;
    }

    public double[] setSimPosition(double p_targetPos, double curve) {
        if (p_targetPos > maxtickcount) {
            p_targetPos = maxtickcount;
        }
        if (p_targetPos < mintickcount) {
            p_targetPos = mintickcount;
        }
        position = currentPos;
        targetPos = p_targetPos;
        lastTime = time;
        isSim = true;

        return getTargetMotion(curve);
    }
}