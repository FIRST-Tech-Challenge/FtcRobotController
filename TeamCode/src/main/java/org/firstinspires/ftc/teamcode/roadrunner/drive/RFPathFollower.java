package org.firstinspires.ftc.teamcode.roadrunner.drive;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kDHead;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kDTrans;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kPHead;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kPTrans;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kV;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentVelocity;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;

public class RFPathFollower {
    double tangentOffset = 0, curviness = 0;
    boolean constantHeading = false, isFollowing = false;
    RFTrajectory rfTrajectory;

    public RFPathFollower() {
        rfTrajectory = new RFTrajectory();
    }

    public double[] update() {
        rfTrajectory.updateSegments();
        if (!isFollowing && rfTrajectory.length() != 0) {
            isFollowing = true;
            rfTrajectory.compileSegments();
        }
        double[] powers = {0, 0, 0, 0};
        //update poseStorage
        if(isFollowing()) {
            Pose2d curPos = currentPose;
            Pose2d curVel = currentVelocity;
            Pose2d PIDTargetVelocity = rfTrajectory.getTargetVelocity();
            Pose2d PIDTargetPose = rfTrajectory.getTargetPosition();
            Vector2d transPosError = PIDTargetPose.vec().minus(curPos.vec());
            Vector2d transVelError = PIDTargetVelocity.vec().minus(curVel.vec());
            double headError = rfTrajectory.angleDist(PIDTargetPose.getHeading()+ rfTrajectory.getTangentOffset(), curPos.getHeading());
            double headVelError = PIDTargetVelocity.getHeading() - curVel.getHeading();
            Pose2d FFTargetAcceleration = rfTrajectory.getInstantaneousTargetAcceleration();
            Pose2d FFTargetVelocity = rfTrajectory.getInstantaneousTargetVelocity();
            Pose2d finalTargetVelocity = new Pose2d(FFTargetVelocity.getX() + transPosError.getX() * kPTrans + transVelError.getX() * kDTrans,
                    FFTargetVelocity.getY() + transPosError.getY() * kPTrans + transVelError.getY() * kDTrans,
                    FFTargetVelocity.getHeading() + headError * kPHead + headVelError * kDHead);
            Vector2d rotateTargetVelocity = finalTargetVelocity.vec().rotated(curPos.getHeading());
            Vector2d rotateTargetAccel = FFTargetAcceleration.vec().rotated(curPos.getHeading());
            double pF = rotateTargetVelocity.getX() * kV + rotateTargetAccel.getX() * kA,
                    pS = rotateTargetVelocity.getY() * kV + rotateTargetAccel.getY() * kA,
                    pR = 2 * TRACK_WIDTH * (finalTargetVelocity.getHeading() * kV + FFTargetAcceleration.getHeading() * kA);
            //frontLeft
            powers[0] = pF - pS - pR;
            //backLeft
            powers[1] = pF + pS - pR;
            //frontRight
            powers[2] = pF - pS + pR;
            //backRight
            powers[3] = pF + pS + pR;
        }
        return powers;
    }

    public void setReversed(boolean reversed) {
        if (reversed) {
            tangentOffset = Math.toRadians(180);
        } else {
            tangentOffset = 0;
        }
    }

    public void setTangentOffset(double p_tangentOffset) {
        tangentOffset = p_tangentOffset;
    }

    public void setConstantHeading(boolean p_constantHeading) {
        constantHeading = p_constantHeading;
    }

    public void setCurviness(double p_curviness) {
        curviness = p_curviness;
    }

    public void addWaypoint(RFWaypoint p_waypoint) {
        rfTrajectory.addSegment(new RFSegment(p_waypoint, tangentOffset, constantHeading));
    }

    public void changeEndpoint(RFWaypoint p_endpoint) {
        rfTrajectory.changeEndpoint(p_endpoint);
    }

    public void eraseWaypoints() {
        rfTrajectory.clear();
    }

    public boolean isFollowing() {
        return rfTrajectory.length() != 0;
    }

}
