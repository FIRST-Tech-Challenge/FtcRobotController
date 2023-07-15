package org.firstinspires.ftc.teamcode.roadrunner.drive;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentVelocity;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.ArrayList;

public class CubicHermiteSpline {
    private double t;
    private ArrayList<Vector2d> coeffs;
    private double length = 0;
    private double duration = 0;
    private double lengthResolution = 200;
    private double numericDerivResolution = 10000, travelDist=0;
    private double AVG_SCALE_FACTOR=1.2;
    private RFTrajectory traj;
    private Vector2d endPos, endVel, lastPos;
    Pose2d targetPose;
    Pose2d targetVelocity;
    Pose2d targetAcceleration;
    Pose2d instantaneousVelocity;
    Pose2d instantaneousAcceleration;

    public CubicHermiteSpline(Vector2d p_startPos, Vector2d p_startVel, Vector2d p_endVel, Vector2d p_endPos, RFTrajectory p_traj) {
        traj = p_traj;
        endPos = p_endPos;
        endVel = p_endVel;
        length = p_startPos.distTo(p_endPos);
        //calc approxDuration
        duration = traj.calculateSegmentDuration(length*AVG_SCALE_FACTOR);
        Vector2d p_startVelo = p_startVel.times(duration);
        Vector2d p_endVelo = p_endVel.times(duration);
        coeffs.add(p_startPos);
        coeffs.add(p_startVelo);
        coeffs.add(p_startPos.times(-3).minus(p_startVelo.times(2)).minus(p_endVelo).plus(p_endPos.times(3)));
        coeffs.add(p_startPos.times(2).plus(p_startVelo).plus(p_endVelo).minus(p_endPos.times(2)));
        Vector2d lastPos = p_startPos;
        for (int i = 1; i < lengthResolution; i++) {
            Vector2d newPos = poseAt(i / lengthResolution, coeffs);
            length += Math.sqrt(newPos.distTo(lastPos));
            lastPos = newPos;
        }
        //calc duration
        duration = traj.calculateSegmentDuration(length);
        p_startVelo = p_startVel.times(duration);
        p_endVelo = p_endVel.times(duration);
        coeffs.clear();
        coeffs.add(p_startPos);
        coeffs.add(p_startVelo);
        coeffs.add(p_startPos.times(-3).minus(p_startVelo.times(2)).minus(p_endVelo).plus(p_endPos.times(3)));
        coeffs.add(p_startPos.times(2).plus(p_startVelo).plus(p_endVelo).minus(p_endPos.times(2)));
        lastPos = currentPose.vec();
    }

    public double getLength() {
        return length;
    }

    public double getRemDistance(){
        travelDist+=currentPose.vec().distTo(lastPos);
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

    public double calcAngularVel(Vector2d vel, Vector2d accel) {
        return (accel.getY() / vel.getX() - (accel.getX()) * (vel.getY()))
                / ((vel.getY() * vel.getY()) / (vel.getX() * vel.getX()) + 1);
    }

    public void calculateTargetPoseAt(double p_t) {
        Vector2d pose = poseAt(p_t, coeffs);
        Vector2d deriv = derivAt(p_t, coeffs);
        Vector2d scnDeriv = scndDerivAt(p_t, coeffs);
        double angle = Math.atan2(deriv.getY(), deriv.getX());
        targetPose = new Pose2d(pose, angle);
        Vector2d velo = deriv.times(1 / deriv.norm());
        double ttoTimeRatio = traj.timeToTRatio(deriv.norm());
        double angularVel = calcAngularVel(deriv.div(ttoTimeRatio), scnDeriv.div(ttoTimeRatio * ttoTimeRatio));
        targetVelocity = new Pose2d(velo, angularVel);
        Vector2d accel = scnDeriv.times(1 / deriv.norm());
        Vector2d deriv2 = derivAt(p_t + 1 / numericDerivResolution, coeffs);
        Vector2d scnDeriv2 = scndDerivAt(p_t + 1 / numericDerivResolution, coeffs);
        angularVel -= calcAngularVel(deriv2.div(ttoTimeRatio), scnDeriv2.div(ttoTimeRatio * ttoTimeRatio));
        angularVel *= -1 * numericDerivResolution;
        targetAcceleration = new Pose2d(accel, angularVel);
    }

    public void calculateInstantaneousTargetPose() {
        Vector2d curPos = currentPose.vec();
        Vector2d curVel = currentVelocity.vec();
        double remDuration = traj.remainingSegmentTime(getRemDistance());
        Vector2d curVelo = curVel.times(remDuration);
        Vector2d endVelo = endVel.times(remDuration);
        ArrayList<Vector2d> tempCoeffs = new ArrayList();
        tempCoeffs.add(curPos);
        tempCoeffs.add(curVelo);
        tempCoeffs.add(curPos.times(-3).minus(curVelo.times(2)).minus(endVelo).plus(endPos.times(3)));
        tempCoeffs.add(curPos.times(2).plus(curVelo).plus(endVelo).minus(endPos.times(2)));
        Vector2d accel = tempCoeffs.get(2).div(remDuration * remDuration *0.5);
        double angularVel = calcAngularVel(curVel, accel);
        instantaneousVelocity = new Pose2d(curVel, angularVel);
        Vector2d curVel2 = derivAt(1 / numericDerivResolution, tempCoeffs).div(remDuration);
        Vector2d accel2 = scndDerivAt(1 / numericDerivResolution, tempCoeffs).div(remDuration * remDuration);
        angularVel -= calcAngularVel(curVel2, accel2);
        angularVel *= -1 * numericDerivResolution;
        instantaneousAcceleration = new Pose2d(accel, angularVel);
    }

}
