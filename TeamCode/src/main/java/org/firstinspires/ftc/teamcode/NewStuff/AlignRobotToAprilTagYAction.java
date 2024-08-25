package org.firstinspires.ftc.teamcode.NewStuff;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class AlignRobotToAprilTagYAction extends Action {

    DriveTrain driveTrain;
    FieldPosition fieldPosition;
    VisionPortalProcessor visionPortalProcessor;

    List<AprilTagDetection> myAprilTagDetections;

    MoveRobotStraightInchesAction straight;

    public AlignRobotToAprilTagYAction(Action dependentAction, FieldPosition fieldPosition, DriveTrain driveTrain, VisionPortalProcessor visionPortalProcessor) {
        this.dependentAction = dependentAction;
        this.driveTrain = driveTrain;
        this.fieldPosition = fieldPosition;
        this.visionPortalProcessor = visionPortalProcessor;
    }

    public AlignRobotToAprilTagYAction(FieldPosition fieldPosition, DriveTrain driveTrain, VisionPortalProcessor visionPortalProcessor) {
        this.dependentAction = new DoneStateAction();
        this.driveTrain = driveTrain;
        this.fieldPosition = fieldPosition;
        this.visionPortalProcessor = visionPortalProcessor;
    }

    public double getDistanceToMoveY () {
        double distanceToBoard;
        double avgDistanceToBoard = 0;

        myAprilTagDetections = visionPortalProcessor.getAprilTagProcessor().getDetections();

        for (AprilTagDetection detection : myAprilTagDetections) {
            distanceToBoard = detection.ftcPose.y;
            avgDistanceToBoard += distanceToBoard;
        }

        avgDistanceToBoard /= myAprilTagDetections.size();

        return avgDistanceToBoard;
    }

    @Override
    boolean checkDoneCondition() {
        return straight.getIsDone();
    }

    @Override
    void update() {
        if (!hasStarted) {
            visionPortalProcessor.getVisionPortal().setProcessorEnabled(visionPortalProcessor.getAprilTagProcessor(), false);
            straight = new MoveRobotStraightInchesAction(getDistanceToMoveY(), driveTrain);
        }

        straight.update();
    }
}
