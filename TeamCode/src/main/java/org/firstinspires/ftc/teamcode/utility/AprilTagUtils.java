package org.firstinspires.ftc.teamcode.utility;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

// Utility function to convert apriltag detections into robot field pose
public class AprilTagUtils {

    // apriltag library for IntoTheDeep
    static AprilTagLibrary tagLibrary = AprilTagGameDatabase.getIntoTheDeepTagLibrary();

    // camera positions relative to center of robot
    // (x,y) in inches!
    static Pose2d CamPositions[] = {
            new Pose2d(4.0, 0.0, new Rotation2d(Math.toRadians(0.0))),      // camera #0
    };


    // calculate and return current robot field Pose given AprilTag detection data
    // input apriltag detection info
    // camerid - id of camera - 0-indexed value
    // NOTE: returns robot pose in inches! Caller will need to convert to other units as req'd
    public static Pose2d CalculateRobotFieldPose (AprilTagDetection detection, int CameraId)
    {
        // get apriltag data for detected tag - subtract 90deg to convert to FTClib orientation
        AprilTagMetadata tag = tagLibrary.lookupTag(detection.id);
        Orientation orientation = tag.fieldOrientation.toOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        double tagZAngle = orientation.thirdAngle-90.0;


        ////////////////
        // Step #1 - determine vector from april tag to robot center in Camera Frame (CF) coordinates
        Translation2d AprilTagCF = new Translation2d(detection.ftcPose.y,
                -detection.ftcPose.x);
        Translation2d RobotCenterCF = (new Translation2d(-CamPositions[CameraId].getX(),
                -CamPositions[CameraId].getY()))
                .rotateBy(CamPositions[CameraId].getRotation().unaryMinus());
        Translation2d ATtoRobotCenterCF = RobotCenterCF.minus(AprilTagCF);

        // Robot angle in camera frame
        Rotation2d RobotAngleCF = new Rotation2d (-CamPositions[CameraId].getRotation().getRadians());

        ////////////////
        // Step #2: Robot position in AprilTag [coordinate] Frame (AF)
        Translation2d ATtoRobotCenterAF = ATtoRobotCenterCF.rotateBy(new Rotation2d(Math.toRadians(-detection.ftcPose.yaw)));

        // robot angle in apriltag frame
        Rotation2d RobotAngleAF = RobotAngleCF.rotateBy(new Rotation2d (Math.toRadians(-detection.ftcPose.yaw)));


        ////////////////
        // Step #3: robot position in Field [coordinate] Frame (FF)
        Translation2d ATtoRobotCenterFF = (ATtoRobotCenterAF.rotateBy(new Rotation2d (Math.toRadians(180.0+tagZAngle))))
                .plus(new Translation2d(tag.fieldPosition.get(0), tag.fieldPosition.get(1)));

        // robot angle in field frame
        Rotation2d RobotAngleFF = RobotAngleAF.rotateBy(new Rotation2d (Math.toRadians(180.0+tagZAngle)));


        // return robot position and angle in a single Pose2d object
        return new Pose2d(ATtoRobotCenterFF, RobotAngleFF);
    }


}