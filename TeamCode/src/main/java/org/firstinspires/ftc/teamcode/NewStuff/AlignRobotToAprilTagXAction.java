package org.firstinspires.ftc.teamcode.NewStuff;

import android.util.Log;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AlignRobotToAprilTagXAction extends Action {

    DriveTrain driveTrain;
    FieldPosition fieldPosition;
    VisionPortalProcessor visionPortalProcessor;

    double distanceBetweenId = 6;

    List<AprilTagDetection> myAprilTagDetections;

    MecanumRobotAction mecanum;

    public AlignRobotToAprilTagXAction(Action dependentAction, FieldPosition fieldPosition, DriveTrain driveTrain, VisionPortalProcessor visionPortalProcessor) {
        this.dependentAction = dependentAction;
        this.driveTrain = driveTrain;
        this.fieldPosition = fieldPosition;
        this.visionPortalProcessor = visionPortalProcessor;
    }

    public AlignRobotToAprilTagXAction(FieldPosition fieldPosition, DriveTrain driveTrain, VisionPortalProcessor visionPortalProcessor) {
        this.dependentAction = new DoneStateAction();
        this.driveTrain = driveTrain;
        this.fieldPosition = fieldPosition;
        this.visionPortalProcessor = visionPortalProcessor;
    }

    public double getDistanceToMoveX () {
        double distanceToMove;
        double avgDistanceToMove = 0;

        myAprilTagDetections = visionPortalProcessor.getAprilTagProcessor().getDetections();

        if (myAprilTagDetections.size() == 0) {
            Log.d("alignx", "no tags detected");
        }

        if (myAprilTagDetections.size() != 0) {
            Log.d("alignx", "tags detected");
            Log.d("alignx", "detections size is " + myAprilTagDetections.size());

            for (AprilTagDetection detection : myAprilTagDetections) {
                distanceToMove = ((fieldPosition.wantedAprTagId - detection.id) * distanceBetweenId) - detection.ftcPose.x;
                Log.d("alignx", "distance to move is " + distanceToMove);
                avgDistanceToMove += distanceToMove;
            }
        }

        avgDistanceToMove /= myAprilTagDetections.size();
        Log.d("alignx", "avg distance to move is " + avgDistanceToMove);

        return avgDistanceToMove;
    }

    @Override
    boolean checkDoneCondition() {
        return mecanum.getIsDone();
    }

    @Override
    void update() {
        if (!hasStarted) {
            visionPortalProcessor.getVisionPortal().setProcessorEnabled(visionPortalProcessor.getAprilTagProcessor(), true);
            mecanum = new MecanumRobotAction(getDistanceToMoveX(), driveTrain);
            hasStarted = true;
        }

        mecanum.update();
    }
}
