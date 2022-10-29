package org.firstinspires.ftc.teamcode.util.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class OdometryPod {
    public static final double ticksPerRev = 8192;
    public static final double circumference = 6.283;
    public static final double inchesPerTick = circumference/ticksPerRev;
    public DcMotor pod;
    private int lastPos;


    public OdometryPod(HardwareMap hardwareMap, String podName) {
        pod = hardwareMap.get(DcMotor.class, podName);
        lastPos = pod.getCurrentPosition();
    }
    public double getDistanceChangeInches() {
        double distanceChange = inchesPerTick * (pod.getCurrentPosition() - lastPos);
        lastPos = pod.getCurrentPosition();
        return distanceChange;
    }
}
