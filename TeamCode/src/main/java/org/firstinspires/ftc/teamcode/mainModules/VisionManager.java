package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.maps.AprilTag;
import org.firstinspires.ftc.teamcode.maps.AprilTagMapping;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class VisionManager {

    private final ElapsedTime clock = new ElapsedTime();
    private double elapsedTime = 0;

    private OnBoardVision onBoardVision;
    private ExternalVision externalVision;

    private Map<Integer, AprilTag> aprilTags = AprilTagMapping.getMap();
    private List<AprilTagDetection> aprilTagDetections = null;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private boolean onBoardInitError = false;
    private boolean externalInitError = false;

    public void initVision(HardwareMap hardwareMapPorted, Telemetry telemetryPorted) {
        hardwareMap = hardwareMapPorted;
        telemetry = telemetryPorted;

        onBoardVision = new OnBoardVision();
        try {
            onBoardVision.initProcessor(hardwareMap, telemetry);
        } catch (Exception e1) {
            onBoardInitError = true;
            telemetry.addData("OnBoard Vision Init Error", e1.getMessage());
        }

        externalVision = new ExternalVision();
        try {
            externalVision.initExternalVision();
        } catch (Exception e2) {
            externalInitError = true;
            telemetry.addData("External Vision Init Error", e2.getMessage());
        }
    }

    public double[] returnPositionData(boolean forceOnBoardProcessor) {
        boolean isUpdated = false;
        double[] robotPosition = new double[]{-1, -1, -1}; //set deafult value to -1 if not detected because robots position cant be negative
        double poseX = 0; //if 0:0 the gimbal will not move so 0:0 is the deafult return
        double poseY = 0;
        double poseZ = 0;
        double upDatedFrame = 0;
        try {
            if ((clock.milliseconds() - elapsedTime) > 33.333) {
                upDatedFrame = 1;
                elapsedTime = clock.milliseconds();
            }

            if (!forceOnBoardProcessor && !externalInitError) {
                try {
                    aprilTagDetections = externalVision.returnAprilTagData();
                    isUpdated = true;
                } catch (Exception e3) {
                    telemetry.addData("External Vision Error", e3.getMessage());
                }
            }

            if (!isUpdated && !onBoardInitError) {
                try {
                    aprilTagDetections = onBoardVision.returnAprilTagData();
                    isUpdated = true;
                } catch (Exception e4) {
                    telemetry.addData("OnBoard Vision Error", e4.getMessage());
                }
            }

            if (isUpdated && aprilTagDetections != null) {
                for (AprilTagDetection detection : aprilTagDetections) {

                    poseX = detection.ftcPose.x;
                    poseY = detection.ftcPose.y;
                    poseZ = detection.ftcPose.z;

                    robotPosition = calculateRobotPosition();

                }
            }

        } catch (Exception e) {
            telemetry.addData("VisionError", true);

        }
        return new double[]{
                robotPosition[0], robotPosition[1], robotPosition[2],
                poseX, poseY, poseZ, upDatedFrame
        };
    }
    private double[] calculateRobotPosition() {
            // Implement logic to calculate the robot's position
        return new double[]{1, 1, 1};
    }

    //check if some errors passed
    private double checkNaN(double checkAble){
        if (Double.isNaN(checkAble)){
            return -1;
        } else {
            return checkAble;
        }
    }
}
