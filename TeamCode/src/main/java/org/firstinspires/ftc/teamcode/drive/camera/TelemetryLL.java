package org.firstinspires.ftc.teamcode.drive.camera;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@Autonomous
public class TelemetryLL extends OpMode {
    Limelight3A limelight;
    IMU imu;
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0); // Switch to pipeline number 0

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters;
        myIMUparameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(myIMUparameters);
    }
    public void loop(){
        LLResult result = limelight.getLatestResult();
        // First, tell Limelight which way your robot is facing
        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        limelight.updateRobotOrientation(robotYaw);
        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
            for (LLResultTypes.DetectorResult detection : detections) {
                String className = detection.getClassName(); // What was detected
                double x = detection.getTargetXDegrees(); // Where it is (left-right)
                double y = detection.getTargetYDegrees(); // Where it is (up-down)
                telemetry.addData(className, "at (" + x + ", " + y + ") degrees");
        }
        }
    }
}