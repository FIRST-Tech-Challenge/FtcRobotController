package org.firstinspires.ftc.teamcode.roadrunner.drive;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentVelocity;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.ArrayList;

public class RFTrajectory {
    ArrayList<RFSegment> segments;
    CubicHermiteSpline currentPath;
    RFMotionProfile motionProfile;
    RFWaypoint startPoint;
    int segIndex = -1;

    public RFTrajectory() {
        segments = new ArrayList();
        startPoint = new RFWaypoint(currentPose,currentVelocity.vec().angle(),currentVelocity.vec().norm());
    }

    public RFTrajectory(RFTrajectory p_trajectory) {
        for (int i = 0; i < p_trajectory.length(); i++) {
            segments.add(p_trajectory.getSegment(i));
        }
    }

    public void addSegment(RFSegment p_segment) {
        segments.add(p_segment);
        segments.get(segments.size()-2).setCompiled(false);
        segments.get(segments.size()-3).setCompiled(false);
    }

    public void compileSegments(){
        for(int i=0;i<segments.size();i++){
            if(!segments.get(i).isCompiled()){
                CatmulRomInterpolater CRerp = new CatmulRomInterpolater(segments, i);
                segments.get(i).getWaypoint().changeTo(CRerp.CRerpWaypoint());
                segments.get(i).setCompiled(true);
            }
        }
    }

    public Vector2d tangentAndMagToVector(double tangent, double mag) {
        return new Vector2d(cos(tangent) * mag, sin(tangent) * mag);
    }
    public void setCurrentPath(RFWaypoint p0, RFWaypoint p1){
        currentPath = new CubicHermiteSpline(p0.getTarget().vec(), p0.getEndVelocityVec(),
                p1.getEndVelocityVec(), p1.getTarget().vec(),this);
    }
    public void setMotionProfile(RFWaypoint p0, RFSegment p1){
        motionProfile = new RFMotionProfile(p0.getTarget().vec(), p0.getEndVelocityVec(),
                p1.getWaypoint().getEndVelocityVec(), p1.getWaypoint().getTarget().vec(),
                p1.getCurviness());
    }

    public double currentT(){
        //calc current t
            //function to map time to t
                //inverse of cubic, inverse of integral of trapezoid, set equal to each other, solve for t
            //use real time to get t
        return 0;
    }

    public double remainingSegmentTime(){
        //calc remaining distance
            //running sum of distance traveled projected on to spline diff with initial calculated distance
        //convert distance to time
        return 0;
    }

    public void setCurrentSegment(int index){
        segIndex = index;
        if(segIndex>0){
            setCurrentPath(segments.get(segIndex-1).getWaypoint(), segments.get(segIndex).getWaypoint());
            setMotionProfile(segments.get(segIndex-1).getWaypoint(), segments.get(segIndex));
        }
        else{
            RFWaypoint curWaypoint = new RFWaypoint(currentPose,currentVelocity.vec().angle(),currentVelocity.vec().norm());
            setCurrentPath(curWaypoint, segments.get(segIndex).getWaypoint());
            setMotionProfile(curWaypoint, segments.get(segIndex));
        }
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
        segIndex=-1;
    }
}
