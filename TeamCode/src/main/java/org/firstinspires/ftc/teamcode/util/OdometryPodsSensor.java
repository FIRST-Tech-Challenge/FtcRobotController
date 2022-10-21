package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OdometryPodsSensor {
    /**
     * IMPORTANT:
     * This class assumes that there are three odometry pods on the front, left, and right side of the robot.
     **/
    Servo leftPod;
    Servo rightPod;
    Servo frontPod;
    private static final double podCircumference = 6.283;
    private static final double ticksPerRev = 384.5;
    private static final double ticksPerInch = ticksPerRev / podCircumference;


    public OdometryPodsSensor(HardwareMap hardwareMap) {
        leftPod = hardwareMap.get(Servo.class, "odometryLeft");
        rightPod = hardwareMap.get(Servo.class, "odometryRight");
        frontPod = hardwareMap.get(Servo.class, "odometryFront");
    }

    public double getDistanceChangeForward(double startPos) {
        double distanceChangeLeft = leftPod.getPosition() - startPos;
        double distanceChangeright = leftPod.getPosition() - startPos;
        double distanceChangeFront = leftPod.getPosition() - startPos;
        return podCircumference*((distanceChangeLeft + distanceChangeright)/2);
    }

    public double getDistanceChangeX(double startPos) {
        double distanceChangeLeft = leftPod.getPosition() - startPos;
        double distanceChangeright = leftPod.getPosition() - startPos;
        double distanceChangeFront = leftPod.getPosition() - startPos;

    }

}
