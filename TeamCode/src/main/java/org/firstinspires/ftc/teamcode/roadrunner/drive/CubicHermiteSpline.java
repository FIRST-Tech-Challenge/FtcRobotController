package org.firstinspires.ftc.teamcode.roadrunner.drive;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentVelocity;

import static java.lang.Math.abs;
import static java.lang.Math.min;
import static java.lang.Math.pow;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.ArrayList;

public class CubicHermiteSpline {
    private double t;
    private ArrayList<Vector2d> coeffs;
    private double length = 0;
    private double duration = 0;
    private double lengthResolution = 400;
    private double numericDerivResolution = 10000, travelDist=0;
    private double AVG_SCALE_FACTOR=1.2;
     double numericIntegral, numericT=0;
    private double numericIntegralResolution = 0.001;
    private RFTrajectory traj;
    double targetDistance=0;
    Vector2d endPos, endVel,startPos,startVel, lastPos, lastIntegralPos;
    Pose2d targetPose;
    Pose2d targetVelocity;
//    Pose2d targetAcceleration;
    Pose2d instantaneousVelocity;
    Pose2d instantaneousAcceleration;

    public CubicHermiteSpline(Vector2d p_startPos, Vector2d p_startVel, Vector2d p_endVel, Vector2d p_endPos, RFTrajectory p_traj) {
        traj = p_traj;
        startPos=p_startPos;
        startVel=p_startVel;
        endPos = p_endPos;
        endVel = p_endVel;
        length = p_startPos.distTo(p_endPos);
        //calc approxDuration
        packet.put("length0",length);

        duration = traj.calculateSegmentDuration(length*AVG_SCALE_FACTOR);
        Vector2d p_startVelo = p_startVel.times(duration);
        Vector2d p_endVelo = p_endVel.times(duration);
        packet.put("duration",duration);
        coeffs= new ArrayList<>();
        coeffs.add(p_startPos);
        coeffs.add(p_startVelo);
        coeffs.add(p_startPos.times(-3).minus(p_startVelo.times(2)).minus(p_endVelo).plus(p_endPos.times(3)));
        coeffs.add(p_startPos.times(2).plus(p_startVelo).plus(p_endVelo).minus(p_endPos.times(2)));
        Vector2d lastPose = p_startPos;
        length=0;
        for (int i = 0; i < lengthResolution; i++) {
            Vector2d newPos = poseAt((i+1) / lengthResolution, coeffs);
            length += newPos.distTo(lastPose);
            lastPose = newPos;
        }
        packet.put("length1",length);

        //calc duration
//        duration = traj.calculateSegmentDuration(length);
        p_startVelo = p_startVel.times(duration);
        p_endVelo = p_endVel.times(duration);
        coeffs.clear();
        coeffs.add(p_startPos);
        coeffs.add(p_startVelo);
        coeffs.add(p_startPos.times(-3).minus(p_startVelo.times(2)).minus(p_endVelo).plus(p_endPos.times(3)));
        coeffs.add(p_startPos.times(2).plus(p_startVelo).plus(p_endVelo).minus(p_endPos.times(2)));
        lastPose = p_startPos;
        length=0;
        for (int i = 0; i < lengthResolution; i++) {
            Vector2d newPos = poseAt((i+1) / lengthResolution, coeffs);
            length += newPos.distTo(lastPose);
            lastPose = newPos;
        }
        packet.put("coeffs0", coeffs.get(0));
        packet.put("coeffs1", coeffs.get(1));
        packet.put("coeffs2", coeffs.get(2));
        packet.put("coeffs3", coeffs.get(3));

        packet.put("length2",length);
        //calc duration
//        duration = traj.calculateSegmentDuration(length);
        p_startVelo = p_startVel.times(duration);
        p_endVelo = p_endVel.times(duration);
        coeffs.clear();
        coeffs.add(p_startPos);
        coeffs.add(p_startVelo);
        coeffs.add(p_startPos.times(-3).minus(p_startVelo.times(2)).minus(p_endVelo).plus(p_endPos.times(3)));
        coeffs.add(p_startPos.times(2).plus(p_startVelo).plus(p_endVelo).minus(p_endPos.times(2)));
        lastPos = currentPose.vec();
        lastIntegralPos = lastPos;
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
        if(vel.getY()==0){
            vel = new Vector2d(vel.getX(),0.00001);
        }
        if(vel.getX()==0){
            vel= new Vector2d(0.00001,vel.getY());
        }

        return (accel.getY() / vel.getX() - (accel.getX()) * (vel.getY()))
                / ((vel.getY() * vel.getY()) / (vel.getX() * vel.getX()) + 1);
    }

    public double power(double v1, int v2){
        double product = v1;
        for(int i=1;i<v2;i++){
            product*=v1;
        }
        return product;
    }

    public double approximateT(double distance){
        int switchTimes=1;
        boolean lastDirection = true;
        numericT=0;
        numericIntegral=0;
        lastIntegralPos = startPos;
        double error = distance-numericIntegral;
        if(error<0){
            lastDirection=false;
        }
        while(abs(error)>numericIntegralResolution){
            if(error>0){
                if(!lastDirection){
                    lastDirection = true;
                    switchTimes++;
                }
                numericT+=Math.max(power(0.1,switchTimes),numericIntegralResolution);
                Vector2d newPos = poseAt(numericT,coeffs);
                numericIntegral+=lastIntegralPos.distTo(newPos);
                lastIntegralPos = newPos;
            }
            else{
                if(lastDirection){
                    lastDirection = false;
                    switchTimes++;
                }
                numericT-=Math.max(power(0.1,switchTimes),numericIntegralResolution);
                Vector2d newPos = poseAt(numericT,coeffs);
                numericIntegral-=lastIntegralPos.distTo(newPos);
                lastIntegralPos = newPos;
            }
            if(switchTimes>5){
                break;
            }
            error = distance-numericIntegral;
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
        double p_t = min(approximateT(distance),1);
        targetDistance = distance;
        Vector2d pose = poseAt(p_t, coeffs);
        Vector2d deriv = derivAt(p_t, coeffs);
        Vector2d scnDeriv = scndDerivAt(p_t, coeffs);
        double angle = Math.atan2(deriv.getY(), deriv.getX());
        targetPose = new Pose2d(pose, angle);
        Vector2d velo;
        if(deriv.norm()!=0) {
            velo = deriv.times(1 / deriv.norm());
        }
        else{
            velo = deriv;
        }
        double ttoTimeRatio = traj.timeToTRatio(deriv.norm());
        packet.put("timeToTRatio", ttoTimeRatio);
        //calculated target Acceleration, not needed for PID
        Vector2d deriv2 = derivAt(p_t + 1 / numericDerivResolution, coeffs);
        double angle2 = Math.atan2(deriv2.getY(),deriv2.getX());
        double angularVel = (angle2-angle)/(numericDerivResolution*ttoTimeRatio);
        targetVelocity = new Pose2d(velo, angularVel);

    }

    public void calculateInstantaneousTargetPose() {
        Vector2d curPos = currentPose.vec();
        Vector2d curVel = currentVelocity.vec();
        double remDuration = 0;
//        traj.remainingSegmentTime(getRemDistance());
        if(remDuration==0){
            remDuration=0.0001;
        }
        Vector2d curVelo = curVel.times(remDuration);
        Vector2d endVelo = endVel.times(remDuration);
        ArrayList<Vector2d> tempCoeffs = new ArrayList();
        tempCoeffs.add(curPos);
        tempCoeffs.add(curVelo);
        tempCoeffs.add(curPos.times(-3).minus(curVelo.times(2)).minus(endVelo).plus(endPos.times(3)));
        tempCoeffs.add(curPos.times(2).plus(curVelo).plus(endVelo).minus(endPos.times(2)));
        Vector2d accel = tempCoeffs.get(2).div(remDuration * remDuration *0.5);
        Vector2d curVel2 = derivAt(1 / numericDerivResolution, tempCoeffs).div(remDuration);
        Vector2d curVel3 = derivAt(2 / numericDerivResolution, tempCoeffs).div(remDuration);
        double angle2 = Math.atan2(curVel2.getY(),curVel2.getX());
        double angle = Math.atan2(curVel.getY(),curVel.getX());
        double angle3 = Math.atan2(curVel3.getY(),curVel3.getX());
        double dT = remDuration/numericDerivResolution;
        double angularVel = (angle2-angle)/dT;
        double angularVel2 = (angle3-angle2)/dT;
        instantaneousVelocity = new Pose2d(curVel, angularVel);
        angularVel = (angularVel2-angularVel)/dT;
        instantaneousAcceleration = new Pose2d(accel, angularVel);
    }

}
