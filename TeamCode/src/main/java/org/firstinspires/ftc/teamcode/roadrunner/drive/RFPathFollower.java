package org.firstinspires.ftc.teamcode.roadrunner.drive;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.dashboard;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kDHead;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kDTrans;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kPHead;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kPTrans;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kV;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentVelocity;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;

import java.util.ArrayList;
import java.util.FormattableFlags;

@Config
public class RFPathFollower {
    double tangentOffset = 0, curviness = 0;
    boolean constantHeading = false, isFollowing = false;
    RFTrajectory rfTrajectory;
    Pose2d finalTargetVelocity = new Pose2d(0,0,0), PIDTargetVelocity= new Pose2d(0,0,0);
    public static int VERBOSITY = 1;

    public RFPathFollower() {
        rfTrajectory = new RFTrajectory();
    }

    public double[] update() {
        if (!isFollowing && rfTrajectory.length() != 0) {
            rfTrajectory.compileSegments();
            isFollowing = true;
        }
        boolean updated = rfTrajectory.updateSegments();
        double[] powers = {0, 0, 0, 0};
        if (isFollowing()) {
            Pose2d curPos = currentPose;
            Pose2d curVel = currentVelocity;
            PIDTargetVelocity = rfTrajectory.getTargetVelocity();
            Pose2d PIDTargetPose = rfTrajectory.getTargetPosition();
            Vector2d transPosError = PIDTargetPose.vec().minus(curPos.vec());
            Vector2d transVelError = PIDTargetVelocity.vec().minus(curVel.vec());
            double headError = rfTrajectory.angleDist(PIDTargetPose.getHeading() + rfTrajectory.getTangentOffset(), curPos.getHeading());
            double headVelError = PIDTargetVelocity.getHeading() - curVel.getHeading();
            Pose2d FFTargetAcceleration = rfTrajectory.getInstantaneousTargetAcceleration();
            Pose2d FFTargetVelocity = rfTrajectory.getInstantaneousTargetVelocity();
            finalTargetVelocity = new Pose2d(FFTargetVelocity.getX() + transPosError.getX() * kPTrans ,
                    FFTargetVelocity.getY() + transPosError.getY() * kPTrans ,
                    FFTargetVelocity.getHeading() + headError * kPHead);
            Pose2d finalTargetAcceleration = new Pose2d(FFTargetAcceleration.getX()+ transVelError.getX() * kDTrans, FFTargetAcceleration.getY()+ transVelError.getY() * kDTrans,
                    FFTargetAcceleration.getHeading()+ headVelError * kDHead);
            Vector2d rotateTargetVelocity = finalTargetVelocity.vec().rotated(curPos.getHeading());
            Vector2d rotateTargetAccel = finalTargetAcceleration.vec().rotated(curPos.getHeading());
            double pF = rotateTargetVelocity.getX() * kV + rotateTargetAccel.getX() * kA,
                    pS = rotateTargetVelocity.getY() * kV + rotateTargetAccel.getY() * kA,
                    pR = 2 * TRACK_WIDTH * (finalTargetVelocity.getHeading() * kV + finalTargetAcceleration.getHeading() * kA);
            //frontLeft
            powers[0] = pF - pS - pR;
            //backLeft
            powers[1] = pF + pS - pR;
            //frontRight
            powers[2] = pF - pS + pR;
            //backRight
            powers[3] = pF + pS + pR;
            if (VERBOSITY > 0) {
                packet.put("curX", curPos.getX());
                packet.put("curY", curPos.getY());
                packet.put("curH", curPos.getHeading());
                packet.put("curdX", curVel.getX());
                packet.put("curdY", curVel.getY());
                packet.put("curdH", curVel.getHeading());
                packet.put("tarX", PIDTargetPose.getX());
                packet.put("tarY", PIDTargetPose.getY());
                packet.put("tarH", PIDTargetPose.getHeading());
                packet.put("targdX", PIDTargetVelocity.getX());
                packet.put("targdY", PIDTargetVelocity.getY());
                packet.put("targdH", PIDTargetVelocity.getHeading());
                packet.put("vel", PIDTargetVelocity.vec().norm());
                packet.put("FFtargdX", FFTargetVelocity.getX());
                packet.put("FFtargdY", FFTargetVelocity.getY());
                packet.put("FFtargdH", FFTargetVelocity.getHeading());
                packet.put("FFtargddX", FFTargetAcceleration.getX());
                packet.put("FFtargddY", FFTargetAcceleration.getY());
                packet.put("FFtargddH", FFTargetAcceleration.getHeading());
                RFSegment seg = rfTrajectory.getCurrentSegment();
                packet.put("target", seg.getWaypoint().getTarget());
                packet.put("segIndex", rfTrajectory.segIndex);
                packet.put("isFollowing", isFollowing());
                packet.put("mpLength", rfTrajectory.motionProfile.length);
                packet.put("mpVelo1", rfTrajectory.motionProfile.velo1);
                packet.put("mpVelo2", rfTrajectory.motionProfile.velo2);
                packet.put("mpCurviness", rfTrajectory.motionProfile.curviness);
                packet.put("mpJerk", rfTrajectory.motionProfile.c);
                packet.put("spLength", rfTrajectory.currentPath.getLength());
                packet.put("startPos", rfTrajectory.currentPath.startPos);
                packet.put("startVel", rfTrajectory.currentPath.startVel);
                packet.put("endVel", rfTrajectory.currentPath.endVel);
                packet.put("endPos", rfTrajectory.currentPath.endPos);
                packet.put("definedness", seg.getWaypoint().getDefinedness());
                packet.put("endVelMag", seg.getWaypoint().getEndVelocity());
                packet.put("isCompiled", seg.isCompiled());
                for (int i = 0; i < 8; i++) {
                    packet.put("tList" + i, rfTrajectory.motionProfile.tList.get(i));
                }
                ArrayList<Vector2d> derivs = rfTrajectory.currentPath.getCurDerivs();
                packet.put("spPose", derivs.get(0));
                packet.put("spDeriv", derivs.get(1));
                packet.put("spScnDeriv", derivs.get(2));
                packet.put("spTargetDist", rfTrajectory.currentPath.targetDistance);
                packet.put("spT", rfTrajectory.currentPath.numericT);
                packet.put("mpVelMag", rfTrajectory.motionProfile.calculateTargetVelocity(time));
                packet.put("time", time);
                packet.put("motionProfileDone", rfTrajectory.motionProfile.isProfileDone(time));
                packet.put("isFollowing", isFollowing);

            }
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
        rfTrajectory.addSegment(new RFSegment(p_waypoint, tangentOffset, constantHeading, curviness));
    }

    public void changeEndpoint(RFWaypoint p_endpoint) {
        rfTrajectory.changeEndpoint(p_endpoint);
    }

    public void eraseWaypoints() {
        rfTrajectory.clear();
    }

    public boolean isFollowing() {
        if (rfTrajectory.length() == 0) {
            isFollowing = false;
        }
        return rfTrajectory.length() != 0;
    }

}
