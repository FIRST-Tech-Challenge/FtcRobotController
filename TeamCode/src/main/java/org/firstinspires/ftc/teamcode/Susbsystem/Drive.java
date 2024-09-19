package org.firstinspires.ftc.teamcode.Susbsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotClass;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Drive {


    public static double[] wheelSpeedsSixWheel = new double[4];

    public static Map<RobotClass.MOTORS, Double> DriveCartesian(double rotX, double rotY, double rotation){

        Map<RobotClass.MOTORS, Double> wheelSpeedsMecanum = new HashMap<>();

        wheelSpeedsMecanum.put(RobotClass.MOTORS.FRONT_LEFT, rotY + rotX - rotation);
        wheelSpeedsMecanum.put(RobotClass.MOTORS.BACK_LEFT, rotY - rotX - rotation);
        wheelSpeedsMecanum.put(RobotClass.MOTORS.BACK_RIGHT, rotY - rotX + rotation);
        wheelSpeedsMecanum.put(RobotClass.MOTORS.FRONT_RIGHT, rotY + rotX + rotation);
        //return normalizeRanges(wheelSpeedsMecanum);
        return wheelSpeedsMecanum;
    }

    public static Map<RobotClass.MOTORS, Double> normalizeRanges(Map<RobotClass.MOTORS, Double> wheelSpeeds, double x, double y, double rX){

        double maxWheelSpeed = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rX), 1);

        maxWheelSpeed = Math.max(maxWheelSpeed, 1);
        wheelSpeeds.put(RobotClass.MOTORS.FRONT_LEFT, wheelSpeeds.get(RobotClass.MOTORS.FRONT_LEFT) / maxWheelSpeed);
        wheelSpeeds.put(RobotClass.MOTORS.FRONT_RIGHT, wheelSpeeds.get(RobotClass.MOTORS.FRONT_RIGHT) / maxWheelSpeed);
        wheelSpeeds.put(RobotClass.MOTORS.BACK_LEFT, wheelSpeeds.get(RobotClass.MOTORS.BACK_LEFT) / maxWheelSpeed);
        wheelSpeeds.put(RobotClass.MOTORS.BACK_RIGHT, wheelSpeeds.get(RobotClass.MOTORS.BACK_RIGHT) / maxWheelSpeed);
        return wheelSpeeds;
    }

}
