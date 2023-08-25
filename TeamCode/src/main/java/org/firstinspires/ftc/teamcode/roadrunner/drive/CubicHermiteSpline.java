package org.firstinspires.ftc.teamcode.roadrunner.drive;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentVelocity;

import static java.lang.Double.NaN;
import static java.lang.Double.isNaN;
import static java.lang.Double.max;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.min;
import static java.lang.Math.pow;
import static java.lang.Math.signum;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.ArrayList;

/**
 * we go quintic now );
 */
@Config
public class CubicHermiteSpline {
    private double t;
    private ArrayList<Vector2d> coeffs;
    private double length = 0;
    private double duration = 0;
    private double lengthResolution = 50;
    public double numericDerivResolution = 10,    travelDist = 0;
    private double AVG_SCALE_FACTOR = 1;
    double numericIntegral, numericT = 0;
    private double numericIntegralResolution = 0.001;
    private RFTrajectory traj;
    double targetDistance = 0;
    Vector2d endPos, endVel, startPos, startVel, lastPos, lastIntegralPos;
    Pose2d targetPose = currentPose;
    Pose2d targetVelocity = currentVelocity;
    Pose2d targetAcceleration = new Pose2d(0,0,0);
    //    Pose2d targetAcceleration;
    Pose2d instantaneousVelocity;
    Pose2d instantaneousAcceleration;
    Vector2d curVel2;
    public static double LOOPINESS = 1, HEADING_ALIGN_TIME = 0.2, END_VEL_SCALE = 1.0;
    public boolean isOverride = false;

    public CubicHermiteSpline(Vector2d p_startPos, Vector2d p_startVel, Vector2d p_endVel, Vector2d p_endPos, RFTrajectory p_traj) {
        traj = p_traj;
        startPos = p_startPos;
        startVel = p_startVel;
        endPos = p_endPos;
        endVel = p_endVel;
        length = p_startPos.distTo(p_endPos);
//        if(traj.getCurrentSegment().getTangentOffset()!=0){
        double startMag = p_startVel.norm();
        double angle = currentPose.getHeading() + traj.getCurrentSegment().getTangentOffset();
        p_startVel = new Vector2d(cos(angle), sin(angle)).times(startMag);
//        }
        //calc approxDuration
        packet.put("length0", length);
        double loopyStorage = LOOPINESS;
        duration = traj.calculateSegmentDuration(length * AVG_SCALE_FACTOR);
        if(endVel.norm()>2) {
            endVel = endVel.times(min(traj.getEndVelMag(), endVel.norm()) / endVel.norm());
        }
        Vector2d p_startVelo = p_startVel.times(LOOPINESS*duration);
        Vector2d p_endVelo = endVel.times(duration);


        packet.put("duration", duration);
        coeffs = new ArrayList<>();
        coeffs.add(p_startPos);
        coeffs.add(p_startVelo);
        coeffs.add(p_startPos.times(-3).minus(p_startVelo.times(2)).minus(p_endVelo).plus(p_endPos.times(3)));
        coeffs.add(p_startPos.times(2).plus(p_startVelo).plus(p_endVelo).minus(p_endPos.times(2)));
        Vector2d lastPose = p_startPos;
        length = 0;
        for (int i = 0; i < lengthResolution; i++) {
            Vector2d newPos = poseAt((i + 1) / lengthResolution, coeffs);
            length += newPos.distTo(lastPose);
            lastPose = newPos;
        }
        packet.put("length1", length);
        //calc duration
        duration = traj.calculateSegmentDuration(length);
        packet.put("duration1", duration);

        double newEndVel = traj.getEndVelMag();
        packet.put("newEndVel", newEndVel);

        if(endVel.norm()>2) {
            endVel = endVel.times(min(traj.getEndVelMag(), endVel.norm()) / endVel.norm());
        }
         p_startVelo = p_startVel.times(LOOPINESS*duration);
         p_endVelo = endVel.times(duration);
        if (p_endVelo.norm() != 0) {
            p_endVelo = p_endVelo.times(newEndVel / p_endVelo.norm());
        }
        coeffs.clear();
        coeffs.add(p_startPos);
        coeffs.add(p_startVelo);
        coeffs.add(p_startPos.times(-3).minus(p_startVelo.times(2)).minus(p_endVelo).plus(p_endPos.times(3)));
        coeffs.add(p_startPos.times(2).plus(p_startVelo).plus(p_endVelo).minus(p_endPos.times(2)));
        lastPose = p_startPos;
        length = 0;
        for (int i = 0; i < lengthResolution; i++) {
            Vector2d newPos = poseAt((i + 1) / lengthResolution, coeffs);
            length += newPos.distTo(lastPose);
            lastPose = newPos;
        }
        packet.put("coeffs0", coeffs.get(0));
        packet.put("coeffs1", coeffs.get(1));
        packet.put("coeffs2", coeffs.get(2));
        packet.put("coeffs3", coeffs.get(3));
        //calc duration
        duration = traj.calculateSegmentDuration(length);
        if(endVel.norm()>2) {
            endVel = endVel.times(min(traj.getEndVelMag(), endVel.norm()) / endVel.norm());
        }
        p_startVelo = p_startVel.times(LOOPINESS*duration);
        p_endVelo = endVel.times(duration);

        if (p_endVelo.norm() != 0) {
            p_endVelo = p_endVelo.times(newEndVel / p_endVelo.norm());
        }
        coeffs.clear();
        coeffs.add(p_startPos);
        coeffs.add(p_startVelo);
        coeffs.add(p_startPos.times(-3).minus(p_startVelo.times(2)).minus(p_endVelo).plus(p_endPos.times(3)));
        coeffs.add(p_startPos.times(2).plus(p_startVelo).plus(p_endVelo).minus(p_endPos.times(2)));
        lastPos = currentPose.vec();
        lastIntegralPos = lastPos;
//        numericDerivResolution*=duration;
        LOOPINESS = loopyStorage;
        travelDist = 0;
    }

    public double getLength() {
        return length;
    }

    public double getRemDistance() {
        travelDist += currentPose.vec().distTo(lastPos);
        lastPos = currentPose.vec();
        return max(length - travelDist, abs(currentPose.vec().distTo(traj.getCurrentSegment().getWaypoint().getTarget().vec())));
    }
    public double internalGetRemDistance() {
        travelDist += currentPose.vec().distTo(lastPos);
        lastPos = currentPose.vec();
        return length - travelDist;
    }

    public Vector2d poseAt(double p_t, ArrayList<Vector2d> coeffs) {
        return coeffs.get(0).plus(coeffs.get(1).times(p_t)).plus(coeffs.get(2).times(p_t * p_t)).plus(coeffs.get(3).times(p_t * p_t * p_t));
    }

    public Vector2d derivAt(double p_t, ArrayList<Vector2d> coeffs) {
        return coeffs.get(1).plus(coeffs.get(2).times(2 * p_t)).plus(coeffs.get(3).times(3 * p_t * p_t));
    }

    public Vector2d scndDerivAt(double p_t, ArrayList<Vector2d> coeffs) {
        return coeffs.get(2).times(2).plus(coeffs.get(3).times(6 * p_t));
    }

    public double power(double v1, int v2) {
        double product = v1;
        for (int i = 1; i < v2; i++) {
            product *= v1;
        }
        return product;
    }

    public double approximateT(double distance) {
        int switchTimes = 0;
        boolean lastDirection = true;
        numericT = 0;
        numericIntegral = 0;
        lastIntegralPos = startPos;
        double error = distance - numericIntegral;
        if (error < 0) {
            lastDirection = false;
        }
        while (abs(error) > numericIntegralResolution) {
            if (error > 0) {
                if (!lastDirection) {
                    lastDirection = true;
                    switchTimes++;
                }
                numericT += Math.max(0.05*power(0.3, switchTimes), numericIntegralResolution);
                Vector2d newPos = poseAt(numericT, coeffs);
                numericIntegral += lastIntegralPos.distTo(newPos);
                lastIntegralPos = newPos;
            } else {
                if (lastDirection) {
                    lastDirection = false;
                    switchTimes++;
                }
                numericT -= Math.max(power(0.1, switchTimes), numericIntegralResolution);
                Vector2d newPos = poseAt(numericT, coeffs);
                numericIntegral -= lastIntegralPos.distTo(newPos);
                lastIntegralPos = newPos;
            }
            if (switchTimes > 4) {
                break;
            }
            error = distance - numericIntegral;
        }
        return numericT;
    }

    public ArrayList<Vector2d> getCurDerivs() {
        double p_t = approximateT(targetDistance);
        Vector2d pose = poseAt(p_t, coeffs);
        Vector2d deriv = derivAt(p_t, coeffs);
        Vector2d scnDeriv = scndDerivAt(p_t, coeffs);
        ArrayList<Vector2d> derivs = new ArrayList<>();
        derivs.add(pose);
        derivs.add(deriv);
        derivs.add(scnDeriv);
        return derivs;
    }

    public void calculateTargetPoseAt(double distance) {
        double numericDerivResolution = this.numericDerivResolution * duration;

        double p_t = max(min(approximateT(distance), 1), 0)+0.05/duration;
        packet.put("DIst", distance);
        packet.put("approxT", p_t);
        targetDistance = distance;
        Vector2d pose = poseAt(p_t, coeffs);
        Vector2d deriv = derivAt(p_t, coeffs);
        double angle = atan2(deriv.getY(),deriv.getX());
        if(traj.motionProfile.motionProfileRemainingTime(time)<0.5){
            angle = traj.getCurrentSegment().getWaypoint().getTarget().getHeading();
        }
        targetPose = new Pose2d(pose, angle + traj.getCurrentSegment().getTangentOffset());
        double ttoTimeRatio = traj.timeToTRatio(deriv.norm());
        packet.put("timeToTRatio", ttoTimeRatio);
        packet.put("derivX", deriv.getX());
        //calculated target Acceleration, not needed for PID
        double newT = p_t + 1 / numericDerivResolution;
        if (newT > 1) {
            newT = 1;
            numericDerivResolution = 1 / (1 - p_t);
        }
        Vector2d deriv2 = derivAt(newT, coeffs);
        double angle2 = Math.atan2(deriv2.getY(), deriv2.getX());
//        double angularVel2 = (angle2 - angle) * numericDerivResolution * ttoTimeRatio;
//        if (abs(angularVel2) > MAX_ANG_VEL) {
//            angularVel2 *= MAX_ANG_VEL / abs(angularVel2);
//        }
//        if (Double.isNaN(angularVel2)) {
//            angularVel2 = 0;
//        }
        Vector2d scnDeriv = scndDerivAt(newT, coeffs);

        targetVelocity = new Pose2d(deriv, currentVelocity.getHeading());
        angle2 = Math.atan2(deriv2.getY(), deriv2.getX());
         angle = currentPose.getHeading() + traj.getCurrentSegment().getTangentOffset();
//        double angle3 = Math.atan2(curVel3.getY(), curVel3.getX());
        if(traj.motionProfile.motionProfileRemainingTime(time)<0.5){
            angle2 = traj.getCurrentSegment().getWaypoint().getTarget().getHeading();
        }
        double angularDist = distBetween(angle2, angle);
        double angleVel = currentVelocity.getHeading();
        if(angularDist==0){
            angularDist=0.0001;
        }
        double angularVel = -angularDist/abs(angularDist)* MAX_ANG_ACCEL*HEADING_ALIGN_TIME;
        if(angleVel * angleVel/MAX_ANG_ACCEL - 0.5*angleVel/MAX_ANG_VEL*angleVel/MAX_ANG_ACCEL < abs(angularDist)){
            angularVel *= -1;
        }

        angularVel /= HEADING_ALIGN_TIME;

        if(abs(angularVel)<0.0001){
            angularVel = 0.0001;
        }//        double angularVel2 = (distBetween(angle3,angle2)) * this.numericDerivResolution;
        if (abs(angularVel) > MAX_ANG_VEL) {
            angularVel *= MAX_ANG_VEL / abs(angularVel);
        }
        targetAcceleration = new Pose2d(scnDeriv, angularVel);
//        double magSquared = targetVelocity.vec().norm() * targetVelocity.vec().norm();
//        double angleMagSquared = targetVelocity.getHeading() * targetVelocity.getHeading() * TRACK_WIDTH * TRACK_WIDTH * 0.25;
//        double ratio = targetVelocity.vec().norm() / sqrt(magSquared + angleMagSquared);
//        if (ratio < 1) {
//            targetVelocity.times(ratio);
//        }
//        packet.put("ratio",ratio);
    }

    public void calculateInstantaneousTargetPose(double targetVelo) {
        Vector2d curPos = currentPose.vec();
        Vector2d curVel = currentVelocity.vec();
        Pose2d targPos = traj.getCurrentSegment().getWaypoint().getTarget();
        double splineLength = 0;

//        double startMag = curVel.norm();
//        double ang = currentPose.getHeading()+traj.getCurrentSegment().getTangentOffset();
//        curVel= new Vector2d(cos(ang),sin(ang)).times(startMag);
//        double remDistance = max(internalGetRemDistance(), /*targPos.vec().distTo(currentPose.vec())*/0);
//        boolean tooFar = false;
//        if (remDistance == targPos.vec().distTo(currentPose.vec())) {
//            tooFar = true;
//        }
//        double remDuration = traj.remainingSegmentTime(remDistance);
//        packet.put("remDuration", remDuration);
//        if (remDuration < 0.0) {
//            remDuration = 0.01;
//        }
//        double numericDerivResolution = this.numericDerivResolution * remDuration;
////        Vector2d endVel = this.endVel;
////        if(endVel.norm()!=0) {
////            endVel = this.endVel.times(traj.getEndVelMag() / this.endVel.norm());
////        }
        Vector2d curVelo = curVel;
//        if(curPos.distTo(startPos)>15){
//            curVelo = curVelo.times(targetVelo/max(curVel.norm(),0.0001));
//        }
//        remDuration = 1.0001;
        double remDuration = 1;
        curVelo = curVelo.times(remDuration);
        Vector2d endVelo = new Vector2d(0, 0);
//        endVelo = endVel.times(remDuration);
        if (endVel.norm() == 0 || Double.isNaN(endVel.norm()) || isNaN(traj.getEndVelMag())) {
            endVelo = new Vector2d(0, 0);
        } else {
//            remDuration=1;
            if (endVel.norm() > 0.1) {
                endVelo = endVel.times(remDuration);
            }
//            endVelo = endVelo.div(remDuration);
        }
//        if(Double.isNaN(endVelo.norm())){
//            endVelo = new Vector2d(0,0);
//        }
        if (endVelo.norm() < 0.1) {
            endVelo = new Vector2d(0.1, 0.1);
        }
        packet.put("endVeloGam", endVelo);

//        endVelo = new Vector2d(0.1,0.1);
        ArrayList<Vector2d> tempCoeffs = new ArrayList<>();
        tempCoeffs.add(curPos);
        tempCoeffs.add(curVelo);
        tempCoeffs.add(curPos.times(-3).minus(curVelo.times(2)).minus(endVelo).plus(endPos.times(3)));
        tempCoeffs.add(curPos.times(2).plus(curVelo).plus(endVelo).minus(endPos.times(2)));
        Vector2d accel = tempCoeffs.get(2).times(2);
        double newT = 1 / numericDerivResolution;
        if (newT > 1.0) {
            newT = 1.0;
            numericDerivResolution = 0.5;
        }
//        newT = 1.0;
//        curVel = derivAt(0,tempCoeffs).div(remDuration);
        curVel2 = derivAt(newT, tempCoeffs);
//        Vector2d curVel3 = derivAt(2*newT, tempCoeffs).div(remDuration);
        double angle2 = Math.atan2(curVel2.getY(), curVel2.getX());
        double angle = currentPose.getHeading() + traj.getCurrentSegment().getTangentOffset();
//        double angle3 = Math.atan2(curVel3.getY(), curVel3.getX());
        double angularDist = distBetween(angle2, angle);
        double angleVel = currentVelocity.getHeading();
        double angularVel = -angularDist/abs(angularDist)* MAX_ANG_ACCEL*HEADING_ALIGN_TIME;
        if(angleVel * angleVel/MAX_ANG_ACCEL - 0.5*angleVel/MAX_ANG_VEL*angleVel/MAX_ANG_ACCEL < abs(angularDist)){
            angularVel *= -1;
        }
        if(abs(angularDist)<toRadians(15)){
            angularVel = -angleVel/HEADING_ALIGN_TIME;
        }
        angularVel /= HEADING_ALIGN_TIME;

        if(abs(angularVel)<0.0001){
        angularVel = 0.0001;
    }//        double angularVel2 = (distBetween(angle3,angle2)) * this.numericDerivResolution;
        if (abs(angularVel) > MAX_ANG_VEL) {
            angularVel *= MAX_ANG_VEL / abs(angularVel);
        }
        instantaneousVelocity = currentVelocity;
        packet.put("angull", angle);
        packet.put("angull2", angle2);
//        packet.put("angull3", angle3);
        packet.put("angullVel1", angularVel);
//        packet.put("angullVel2", angularVel2);

        packet.put("angullAccel", angularVel);
        packet.put("newT", newT);
        double angleDiff = PI * 0.5;
        curVel = currentVelocity.vec();
//        if(curVel.norm()>0.1&&curVel2.norm()>0.1){
//            curVelo = curVel.times(1/curVel.norm());
//            Vector2d curVelo2 = curVel2.times(1/curVel2.norm());
//            angleDiff = abs(distBetween(curVelo.angle(),curVelo2.angle()));
//            angleDiff = min(angleDiff, PI*0.5);
//            if(curVel.norm()<2||angleDiff>5*PI/180){
//                angleDiff=PI*0.5;
//            }
//        }
//        angularVel = 0.00001;

//        if(instantaneousAcceleration.vec().norm()<5){
//            instantaneousAcceleration = new Pose2d(currentVelocity.vec().times(1.1).minus(curVel), 0.0002);
//            packet.put("FFAccelMag",20);
//
//        }
//        else{
//            packet.put("FFAccelMag", instantaneousAcceleration.vec().norm());
//        }
//        curVel = curVel.times(0.8);
//        if (remDuration<1.0) {
//            curVel2 = targPos.vec().minus(currentPose.vec());
//            targetVelo = abs(targetVelo);
//            double angleDist = distBetween(targPos.getHeading(), currentPose.getHeading());
//            angularVel = -signum(angleDist)* MAX_ANG_ACCEL*HEADING_ALIGN_TIME;
//            if(angleVel * angleVel/MAX_ANG_ACCEL - 0.5*angleVel/MAX_ANG_VEL*angleVel/MAX_ANG_ACCEL < abs(angleDist)){
//                angularVel *= -1;
//            }
//            if(abs(angleDist)<toRadians(15)){
//                angularVel = -angleVel/HEADING_ALIGN_TIME;
//            }
//            angularVel /= HEADING_ALIGN_TIME;
//        }
        if(abs(angularVel)<0.0001){
            angularVel = 0.0001 ;
        }
//        angularVel = max(angularVel,0.0001);

//        curVel2 = curVel2.times(targetVelo/max(curVel2.norm(),0.0001));

//        if (remDuration<1.0) {
//            curVel2 = targPos.vec().minus(currentPose.vec());
//        }
//        if (abs(distBetween(curVel.angle(), curVel2.angle())) < toRadians(1 / this.numericDerivResolution)) {
//            instantaneousAcceleration = new Pose2d(curVel2, angularVel);
//            isOverride = true;
//        } else {
//            isOverride = false;
//        }
        instantaneousAcceleration = new Pose2d(scndDerivAt(0,tempCoeffs), angularVel);

//        instantaneousAcceleration = new Pose2d(curVel2, angularVel);
        packet.put("targetAngle", targPos.getHeading());
        packet.put("curVel2", curVel2.angle());
        packet.put("curVel1", curVel.getY());
        packet.put("accel1", instantaneousAcceleration.getY());
        packet.put("velDiff", curVel2.getY() - curVel.getY() * (sin(angleDiff)));
        packet.put("endVel", endVel.angle());
        packet.put("tempCoeffs1", tempCoeffs.get(1).getX());
        packet.put("tempCoeffs2", tempCoeffs.get(2).getX());
        packet.put("tempCoeffs3", tempCoeffs.get(3).getX());

    }

    public double distBetween(double angle1, double angle2) {
        double dist = angle1 - angle2;
        while (abs(dist) > toRadians(180)) {
            if (dist > toRadians(180)) {
                dist -= toRadians(360);
            } else if (dist < toRadians(-180)) {
                dist += toRadians(360);
            }
        }
        return dist;
    }

}
