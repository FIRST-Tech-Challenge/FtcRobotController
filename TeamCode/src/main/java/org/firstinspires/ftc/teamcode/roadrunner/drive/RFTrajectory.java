package org.firstinspires.ftc.teamcode.roadrunner.drive;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentVelocity;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.ArrayList;

public class RFTrajectory {
    ArrayList<RFSegment> segments;
    CubicHermiteSpline currentPath;
    RFMotionProfile motionProfile;
    RFWaypoint startPoint;
    int segIndex = -1;
    double transAccuracy = 0.5, headingAccuracy = toRadians(30);

    public RFTrajectory() {
        segments = new ArrayList();
        startPoint = new RFWaypoint(currentPose, currentVelocity.vec().angle(), currentVelocity.vec().norm());
    }

    public RFTrajectory(RFTrajectory p_trajectory) {
        for (int i = 0; i < p_trajectory.length(); i++) {
            segments.add(p_trajectory.getSegment(i));
        }
    }

    public void addSegment(RFSegment p_segment) {
        segments.add(p_segment);
        if (segments.size() > 1) {
            segments.get(segments.size() - 2).setCompiled(false);
        }
        if (segments.size() > 2) {
            segments.get(segments.size() - 3).setCompiled(false);
        }
    }

    public Pose2d getTargetPosition() {
        return currentPath.targetPose;
    }

    public boolean updateSegments() {
        if (segIndex == -1) {
            setCurrentSegment(0);
            return true;
        } else if (segIndex + 1 < segments.size()) {
            if (motionProfile.isProfileDone(time)) {
                setCurrentSegment(segIndex + 1);
                return true;

            }
        } else {
            Pose2d curPos = currentPose;
            Pose2d targetPos = segments.get(segments.size() - 1).getWaypoint().getTarget();
            if (curPos.vec().distTo(targetPos.vec()) < 0.25) {
                clear();
                return true;
            }
        }
        return false;
    }

    public double angleDist(double ang1, double ang2) {
        double dist = ang1 - ang2;
        while (dist > Math.toRadians(180)) {
            dist -= Math.toRadians(360);
        }
        while (dist < -Math.toRadians(180)) {
            dist += Math.toRadians(360);
        }
        return dist;
    }

    public double getTangentOffset() {
        return segments.get(segIndex).getTangentOffset();
    }

    //run before getTargetPosition
    public Pose2d getTargetVelocity() {
        double distance = motionProfile.motionProfileTimeToDist(time);
        packet.put("mpPos", distance);
        currentPath.calculateTargetPoseAt(distance);
        return currentPath.targetVelocity;
    }
    //not needed for PID
//    public Pose2d getTargetAcceleration() {
//        return null;
//    }

    public Pose2d getInstantaneousTargetVelocity() {
        return currentPath.instantaneousVelocity;
    }

    //run this before getInstanTaneousTargetVelosity,
    public Pose2d getInstantaneousTargetAcceleration() {
        double accelMag = motionProfile.getInstantaneousTargetAcceleration(currentPath.getRemDistance());
        packet.put("accelMag",accelMag);
        currentPath.calculateInstantaneousTargetPose();
        Pose2d pathAccel = currentPath.instantaneousAcceleration;
        double projectMag = pathAccel.vec().dot(currentVelocity.vec());
        packet.put("projectMag", projectMag);
        if (projectMag == 0) {
            projectMag = 0.0001;
        }
        Vector2d projectedDiff = currentVelocity.vec().times(projectMag / Math.max(currentVelocity.vec().norm(),0.001)).times(1 - (accelMag / projectMag));
        packet.put("pathAccel", pathAccel);
        return new Pose2d(pathAccel.vec().minus(projectedDiff), pathAccel.getHeading());
    }


    public void compileSegments() {
        for (int i = 0; i < segments.size(); i++) {
            if (!segments.get(i).isCompiled()) {
                CatmulRomInterpolater CRerp = new CatmulRomInterpolater(segments, i);
                segments.get(i).getWaypoint().changeTo(CRerp.CRerpWaypoint());
                segments.get(i).setCompiled(true);
            }
        }
    }

    public Vector2d tangentAndMagToVector(double tangent, double mag) {
        return new Vector2d(cos(tangent) * mag, sin(tangent) * mag);
    }

    public void setCurrentPath(RFWaypoint p0, RFWaypoint p1) {
        currentPath = new CubicHermiteSpline(p0.getTarget().vec(), p0.getEndVelocityVec(),
                p1.getEndVelocityVec(), p1.getTarget().vec(), this);
    }

    public void setMotionProfile(RFWaypoint p0, RFSegment p1) {
        motionProfile = new RFMotionProfile(p0.getEndVelocityVec(),
                p1.getWaypoint().getEndVelocityVec(),
                p1.getCurviness());
    }

    public double targetCurrentDistance() {
        return motionProfile.motionProfileTimeToDist(time);
    }

    public double calculateSegmentDuration(double distance) {
        double input = distance;
        motionProfile.setLength(input);
        double output = motionProfile.motionProfileRemainingTime(time);
        return output;
    }

    public double remainingSegmentTime(double distance) {
        motionProfile.motionProfileRemDistToRemTime(distance);
        return 0;
    }

    public double timeToTRatio(double tToXDerivative) {
        double mpVelo = motionProfile.calculateTargetVelocity(time);

        if (tToXDerivative == 0) {
            if (mpVelo == 0) {
                return 1;
            }
        }
        packet.put("veloMag", mpVelo);
        packet.put("segment", segIndex * 10);
        if (tToXDerivative > mpVelo || motionProfile.motionProfileRemainingTime(time) < 0.4) {
            if(mpVelo==0){
                mpVelo = 0.0001;
            }
            if(tToXDerivative ==0){
                tToXDerivative = 0.0001;
            }
            return mpVelo / tToXDerivative;
        } else {
            return mpVelo/ tToXDerivative;
        }
//        return motionProfile.calculateTargetVelocity(time) / tToXDerivative;
    }

    public void setCurrentSegment(int index) {
        segIndex = index;
        Vector2d curVel = currentVelocity.vec();
        if(curVel.norm()==0){
            double angle = currentPose.getHeading();
            curVel = new Vector2d(cos(angle),sin(angle));
        }

        double curAng = currentVelocity.getHeading();
        double velMag = sqrt(currentVelocity.vec().norm()*currentVelocity.vec().norm()+curAng*curAng*TRACK_WIDTH*TRACK_WIDTH*0.25);
        RFWaypoint curWaypoint = new RFWaypoint(currentPose, curVel.angle(), velMag);
        setMotionProfile(curWaypoint, segments.get(segIndex));
        curWaypoint = new RFWaypoint(currentPose, curVel.angle(), curVel.norm());
        setCurrentPath(curWaypoint, segments.get(segIndex).getWaypoint());
    }

    public RFSegment getCurrentSegment() {
        return segments.get(segIndex);
    }

    public RFSegment getSegment(int index) {
        return segments.get(index);
    }

    public void changeEndpoint(RFWaypoint p_newEndpoint) {
        segments.get(segments.size() - 1).getWaypoint().changeTo(p_newEndpoint);
        compileSegments();
    }

    public int length() {
        return segments.size();
    }

    public void clear() {
        segments.clear();
        segIndex = -1;
    }
}
