package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

public class SystemState {
    SparkFunOTOS.Pose2D drivePose;
    armPose armPosition;
    wristState wristPosition;
    clawState clawPosition;
    double ignorePosTime;
}
