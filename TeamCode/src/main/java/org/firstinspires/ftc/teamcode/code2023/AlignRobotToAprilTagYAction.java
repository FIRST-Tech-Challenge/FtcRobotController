package org.firstinspires.ftc.teamcode.code2023;

import android.util.Log;

import org.firstinspires.ftc.teamcode.actions.Action;
import org.firstinspires.ftc.teamcode.actions.DoneStateAction;
import org.firstinspires.ftc.teamcode.actions.MoveRobotStraightInchesAction;
import org.firstinspires.ftc.teamcode.modules.DriveTrain;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class AlignRobotToAprilTagYAction extends Action {

    DriveTrain driveTrain;
    FieldPosition fieldPosition;
    VisionPortalManager visionPortalManager;

    List<AprilTagDetection> myAprilTagDetections;

    MoveRobotStraightInchesAction straight;

    public AlignRobotToAprilTagYAction(Action dependentAction, FieldPosition fieldPosition, DriveTrain driveTrain, VisionPortalManager visionPortalManager) {
        this.dependentAction = dependentAction;
        this.driveTrain = driveTrain;
        this.fieldPosition = fieldPosition;
        this.visionPortalManager = visionPortalManager;
    }

    public AlignRobotToAprilTagYAction(FieldPosition fieldPosition, DriveTrain driveTrain, VisionPortalManager visionPortalManager) {
        this.dependentAction = new DoneStateAction();
        this.driveTrain = driveTrain;
        this.fieldPosition = fieldPosition;
        this.visionPortalManager = visionPortalManager;
    }

    public double getDistanceToMoveY () {
        double distanceToBoard;
        double avgDistanceToBoard = 0;

        myAprilTagDetections = visionPortalManager.getAprilTagProcessor().getDetections();

        if (myAprilTagDetections.size() > 0) {

            for (AprilTagDetection detection : myAprilTagDetections) {
                distanceToBoard = detection.ftcPose.y;
                avgDistanceToBoard += distanceToBoard;
            }

            avgDistanceToBoard /= myAprilTagDetections.size();
            Log.d("yaction", "avg distance is " + avgDistanceToBoard);

        }
        return avgDistanceToBoard;
    }

    @Override
    public boolean checkDoneCondition() {
        if(straight.getIsDone()) {
            driveTrain.setPower(0);
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void update() {
        if (!hasStarted) {
            straight = new MoveRobotStraightInchesAction(-getDistanceToMoveY() + 3, driveTrain);
            hasStarted = true;
        }

        straight.updateCheckDone();

    }
}
