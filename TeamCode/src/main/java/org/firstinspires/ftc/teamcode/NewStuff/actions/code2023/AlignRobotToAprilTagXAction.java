package org.firstinspires.ftc.teamcode.NewStuff.actions.code2023;

import android.util.Log;

import org.firstinspires.ftc.teamcode.NewStuff.FieldPosition;
import org.firstinspires.ftc.teamcode.NewStuff.actions.Action;
import org.firstinspires.ftc.teamcode.NewStuff.actions.DoneStateAction;
import org.firstinspires.ftc.teamcode.NewStuff.actions.MecanumRobotAction;
import org.firstinspires.ftc.teamcode.NewStuff.modules.DriveTrain;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class AlignRobotToAprilTagXAction extends Action {

    DriveTrain driveTrain;
    FieldPosition fieldPosition;
    VisionPortalManager visionPortalManager;

    double distanceBetweenId = 6;

    List<AprilTagDetection> myAprilTagDetections;

    MecanumRobotAction mecanum;

    int counter = 0;

    Boolean hasDetected = false;
    Boolean hasInitialized = false;

    double distanceToMove;

    public AlignRobotToAprilTagXAction(Action dependentAction, FieldPosition fieldPosition, DriveTrain driveTrain, VisionPortalManager visionPortalManager) {
        this.dependentAction = dependentAction;
        this.driveTrain = driveTrain;
        this.fieldPosition = fieldPosition;
        this.visionPortalManager = visionPortalManager;
    }

    public AlignRobotToAprilTagXAction(FieldPosition fieldPosition, DriveTrain driveTrain, VisionPortalManager visionPortalManager) {
        this.dependentAction = new DoneStateAction();
        this.driveTrain = driveTrain;
        this.fieldPosition = fieldPosition;
        this.visionPortalManager = visionPortalManager;
    }

    public void updateDistanceToMoveX () {

        if (counter<20 && !hasDetected) {
            counter++;

            myAprilTagDetections = visionPortalManager.getAprilTagProcessor().getDetections();

            if (myAprilTagDetections.size() == 0) {
                Log.d("alignx", "no tags detected " + counter);
            }

            if (myAprilTagDetections.size() > 0) {

                hasDetected = true;

                Log.d("alignx", "tags detected");
                Log.d("alignx", "detections size is " + myAprilTagDetections.size());

                distanceToMove = ((fieldPosition.getWantedAprTagId() - myAprilTagDetections.get(0).id) * distanceBetweenId) + myAprilTagDetections.get(0).ftcPose.x;
                Log.d("alignx", "distance to move is " + distanceToMove);

            }
        }

    }

    @Override
    public boolean checkDoneCondition() {
        if (hasInitialized) {
            if(mecanum.getIsDone()) {
                driveTrain.setPower(0);
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }

    }

    @Override
    public void update() {
        if (!hasStarted) {
            hasStarted = true;
            visionPortalManager.getVisionPortal().setProcessorEnabled(visionPortalManager.getAprilTagProcessor(), true);
            driveTrain.getOpModeUtilities().getOpMode().sleep(500);
        }

        updateDistanceToMoveX();

        if (hasDetected && !hasInitialized && counter<20) {
            mecanum = new MecanumRobotAction(-distanceToMove, driveTrain);
            hasInitialized = true;
        }

        if (counter>=20 && !hasDetected && !hasInitialized) {
            mecanum = new MecanumRobotAction(0, driveTrain);
            hasInitialized = true;
        }

        if(hasInitialized) {
            mecanum.updateCheckDone();
            Log.d("alignx", "mecanum finished " + mecanum.getIsDone());
        }

    }
}
