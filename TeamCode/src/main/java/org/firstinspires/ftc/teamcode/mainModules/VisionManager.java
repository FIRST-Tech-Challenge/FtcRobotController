package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

public class VisionManager {
    OnBoardVision onBoardVision;
    ExternalVision externalVision;

    ArrayList<AprilTagDetection> aprilTagDetections = null;

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
        } catch (Exception e1){
            onBoardInitError = true;
        }


        externalVision = new ExternalVision();
        try {
            externalVision.initExternalVision();
        } catch (Exception e2){
            externalInitError = true;
        }
    }

    public int[] returnPositionData(boolean forceOnBoardProcessor){

        boolean isUpdated = false;

        if (!forceOnBoardProcessor && !externalInitError){
            try {
                aprilTagDetections = externalVision.returnAprilTagData();
                isUpdated = true;
            } catch (Exception e3){

            }

        }

        if (isUpdated && !onBoardInitError){
            try {
                aprilTagDetections = onBoardVision.returnAprilTagData();
                isUpdated = true;
            } catch (Exception e4){

            }
        }

        if (isUpdated){
            int frameX = 0;
            int frameY = 0;

            int [] robotPosition = calculateRobotPosition();

            for (AprilTagDetection detection : aprilTagDetections) {

            }

            return new int[] {
                    robotPosition[0], robotPosition[0], robotPosition[0],
                    frameX, frameY
                };
        }
        else{
            return new int[]{
                    -1, -1, -1, //robot x, y, absoluteAngle
                    -1, -1 //returns the x, y in the camera frame of the closest aprilTag
            };
        }
    }
    private int[] calculateRobotPosition(){
        return new int[] {0,0,0};
    }
}
