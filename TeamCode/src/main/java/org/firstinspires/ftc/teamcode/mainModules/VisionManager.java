package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.maps.AprilTag;
import org.firstinspires.ftc.teamcode.maps.AprilTagMapping;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class VisionManager {

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

    public int[] returnPositionData(boolean forceOnBoardProcessor) {
        // telemetry.addData("apriltagmap", aprilTags.get(100));
        boolean isUpdated = false;

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
            int frameX = 0;
            int frameY = 0;

            int[] robotPosition = calculateRobotPosition();

            // Process detections to update frameX and frameY if needed
            for (AprilTagDetection detection : aprilTagDetections) {
                // Your logic to update frameX and frameY based on detection
            }

            return new int[]{
                    robotPosition[0], robotPosition[1], robotPosition[2],
                    frameX, frameY
            };
        } else {
            return new int[]{
                    -1, -1, -1, // robot x, y, absoluteAngle
                    -1, -1      // x, y in the camera frame of the closest AprilTag
            };
        }
    }

    private int[] calculateRobotPosition() {
        // Implement logic to calculate the robot's position
        return new int[]{0, 0, 0};
    }
}
