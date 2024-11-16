package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

public class SystemState {
    public SparkFunOTOS.Pose2D drivePose;
    public armPose armPosition;
    public wristState wristPosition;
    public clawState clawPosition;
    public double ignorePosTime;
}
