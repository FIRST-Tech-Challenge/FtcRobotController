package org.firstinspires.ftc.teamcode.util.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OdometryPodsSensor {
    /**
     * IMPORTANT:
     * This class assumes that there are three odometry pods on the front, left, and right side of the robot.
     **/
    private OdometryPod leftPod;
    private OdometryPod rightPod;
    private OdometryPod frontPod;
    private static final double podCircumference = 6.283;
    private static final double ticksPerRev = 8192;
    private static final double ticksPerInch = ticksPerRev / podCircumference;
    private static final double width = 14;
    private static final double frontPodDistance = 14;



    public OdometryPodsSensor(HardwareMap hardwareMap) {
        leftPod = new OdometryPod(hardwareMap, "odometryLeft");
        rightPod = new OdometryPod(hardwareMap, "odometryRight");
        frontPod = new OdometryPod(hardwareMap, "odometeryFront");
    }


    public double[] getStateChange() {
        double distanceChangeLeft  = leftPod.getDistanceChangeInches();
        double distanceChangeRight = rightPod.getDistanceChangeInches();
        double forward = (distanceChangeLeft + distanceChangeRight)/2;
        double headingChange = (distanceChangeLeft - distanceChangeRight) / width; //left - right because we want a positive change.
        double strafe = frontPod.getDistanceChangeInches() - (headingChange*frontPodDistance);//
        double[] retVal = {forward,strafe,headingChange};
        return retVal;
    }

}
