package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

public class SystemState {
    SparkFunOTOS.Pose2D drivePose;
    armPose armPosition;
    wristState wristPosition;
    clawState clawPosition;
    double ignorePosTime;
    public void init(SparkFunOTOS.Pose2D drivePosition, armPose poseOfArm, wristState wristPose, clawState clawPose, double ignorePosTime) {
        armPosition = poseOfArm;
        drivePose = drivePosition;
        wristPosition = wristPose;
        clawPosition = clawPose;
        this.ignorePosTime = ignorePosTime;
    }
}
