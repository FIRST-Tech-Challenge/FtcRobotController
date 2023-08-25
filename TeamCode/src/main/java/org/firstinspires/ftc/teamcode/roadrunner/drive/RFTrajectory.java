package org.firstinspires.ftc.teamcode.roadrunner.drive;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentVelocity;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.RFMecanumDrive.poseMode;

import static java.lang.Double.isNaN;
import static java.lang.Double.max;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.min;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.ArrayList;
@Config
public class RFTrajectory {
    public static double SCALE = 1.0;
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
            if (motionProfile.isProfileDone(time) || (/*currentPose.vec().distTo(segments.get(segIndex).getWaypoint().getTarget().vec()) < 5*/  motionProfile.isProfileDone(time - 1.5))) {
                setCurrentSegment(segIndex + 1);
                return true;

            }
        } else {
            Pose2d curPos = currentPose;
            Pose2d targetPos = segments.get(segments.size() - 1).getWaypoint().getTarget();
            if (curPos.vec().distTo(targetPos.vec()) < 5.0) {
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
        if(poseMode==1||poseMode==3) {
            return currentPath.instantaneousVelocity;
        }
        else{
            return currentPath.targetVelocity.times(motionProfile.targetVelocity/currentPath.targetVelocity.vec().norm());
        }
    }

    //run this before getInstanTaneousTargetVelosity,
    public double projectOnto(Vector2d u, Vector2d v) {
        double dot = u.getX() * v.getX() + u.getX() * v.getY() + u.getY() * v.getY();
        double norm = v.norm();
        if (norm == 0) {
            norm = 0.001;
        }
        return dot / norm;
    }

    //TODO make this work for when u giga slow and out of time
    public Pose2d getInstantaneousTargetAcceleration() {
        if(poseMode==1||poseMode==3) {

            packet.put("currentVelocityVec", currentVelocity.vec());
//        Vector2d deltaVec = currentPath.endPos.minus((currentPose.vec()));
            Vector2d projectedVelo = new Vector2d(0.1, 0);
            if (currentVelocity != null && currentPath.targetVelocity != null) {
                projectedVelo = currentVelocity.vec().projectOnto(currentPath.targetVelocity.vec());
            }

            double accelMag = motionProfile.getInstantaneousTargetAcceleration(max(currentPath.getRemDistance(),
                            currentPose.vec().distTo(getCurrentSegment()
                                    .getWaypoint().getTarget().vec())), projectedVelo
                    , currentPath.endVel);
            packet.put("accelMag", accelMag);
//        if(!currentPath.isOverride){
//            accelMag = abs(accelMag);
//        }

//        packet.put("accelMag",accelMag);
            double targetVeloMag = currentVelocity.vec().norm() + accelMag / currentPath.numericDerivResolution;
            packet.put("targetVeloMag", targetVeloMag);
            packet.put("currentVeloMag", currentVelocity.vec().norm());
//        Vector2d newTargetVel = currentPath.curVel2.times(targetVeloMag / currentPath.curVel2.norm());
//        packet.put("newTargetVel", newTargetVel.getY());
            if (abs(targetVeloMag) < 0.001) {
                targetVeloMag = 0.001;
            }
            currentPath.calculateInstantaneousTargetPose(targetVeloMag);
            Pose2d pathAccel = currentPath.instantaneousAcceleration;
            Vector2d scaledPathAccel = pathAccel.vec().times(abs(accelMag) / pathAccel.vec().norm());
            pathAccel = new Pose2d(scaledPathAccel/*.plus(currentVelocity.vec()).minus(deltaVec)*/, pathAccel.getHeading());
            packet.put("pathAccelMag", pathAccel.vec().norm());
            packet.put("RemDist", max(currentPath.getRemDistance(), currentPose.vec().distTo(getCurrentSegment().getWaypoint().getTarget().vec())));
            return pathAccel;
        }
        else{
            packet.put("currentVelocityVec", currentVelocity.vec());
//        Vector2d deltaVec = currentPath.endPos.minus((currentPose.vec()));

            double accelMag = motionProfile.targetAcceleration;/*motionProfile.getInstantaneousTargetAcceleration(max(currentPath.getRemDistance(),
                            currentPose.vec().distTo(getCurrentSegment()
                                    .getWaypoint().getTarget().vec())), projectedVelo
                    , currentPath.endVel);*/
            packet.put("accelMag", accelMag);
//        if(!currentPath.isOverride){
//            accelMag = abs(accelMag);
//        }

//        packet.put("accelMag",accelMag);
            double targetVeloMag = currentVelocity.vec().norm() + accelMag / currentPath.numericDerivResolution;
            packet.put("targetVeloMag", targetVeloMag);
            packet.put("currentVeloMag", currentVelocity.vec().norm());
//        Vector2d newTargetVel = currentPath.curVel2.times(targetVeloMag / currentPath.curVel2.norm());
//        packet.put("newTargetVel", newTargetVel.getY());
            if (abs(targetVeloMag) < 0.001) {
                targetVeloMag = 0.001;
            }
//            currentPath.calculateInstantaneousTargetPose(targetVeloMag);
            Pose2d pathAccel = currentPath.targetAcceleration;
//            currentPath.targetAcceleration;
            if(pathAccel==null||pathAccel.vec().norm()<0.1){
                pathAccel=new Pose2d(1,0, 0);
            }
            if(isNaN((accelMag))||accelMag==0){
                accelMag=1;
            }
            Vector2d scaledPathAccel = pathAccel.vec().times(SCALE*abs(accelMag) / pathAccel.vec().norm());
            pathAccel = new Pose2d(scaledPathAccel/*.plus(currentVelocity.vec()).minus(deltaVec)*/, pathAccel.getHeading());
            packet.put("pathAccelMag", pathAccel.vec().norm());
//            packet.put("RemDist", max(currentPath.getRemDistance(), currentPose.vec().distTo(getCurrentSegment().getWaypoint().getTarget().vec())));
            return pathAccel;
        }
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

    public double getEndVelMag() {
        return motionProfile.endVel;
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
        return motionProfile.motionProfileRemDistToRemTime(distance);
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
            if (mpVelo == 0) {
                mpVelo = 0.0001;
            }
            if (tToXDerivative == 0) {
                tToXDerivative = 0.0001;
            }
            return mpVelo / tToXDerivative;
        } else {
            return mpVelo / tToXDerivative;
        }
//        return motionProfile.calculateTargetVelocity(time) / tToXDerivative;
    }

    public void setCurrentSegment(int index) {
        segIndex = index;
        Vector2d curVel = currentVelocity.vec();
        if (curVel.norm() == 0) {
            double angle = currentPose.getHeading();
            curVel = new Vector2d(cos(angle), sin(angle));
        }

        double curAng = currentVelocity.getHeading();
        double velMag = sqrt(currentVelocity.vec().norm() * currentVelocity.vec().norm() + curAng * curAng * TRACK_WIDTH * TRACK_WIDTH * 0.25);
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
