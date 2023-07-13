package org.firstinspires.ftc.teamcode.roadrunner.drive;

import com.acmerobotics.roadrunner.trajectory.MarkerCallback;

public class RFPathFollower {
    double tangentOffset=0, curviness=0;
    boolean constantHeading = false, isFollowing = false;
    RFTrajectory rfTrajectory;
    public RFPathFollower(){
        rfTrajectory = null;
    }

    public double[] update(){
        if(!isFollowing&&rfTrajectory.length()!=0){
            isFollowing = true;
            rfTrajectory.compileSegments();
        }
        double[] powers = {0,0,0,0};
        return powers;
    }

    public void setReversed(boolean reversed){
        if(reversed){
            tangentOffset = Math.toRadians(180);
        }
        else{
            tangentOffset = 0;
        }
    }

    public void setTangentOffset(double p_tangentOffset){
        tangentOffset = p_tangentOffset;
    }

    public void setConstantHeading(boolean p_constantHeading){
        constantHeading = p_constantHeading;
    }

    public void setCurviness(double p_curviness){
        curviness = p_curviness;
    }

    public void addWaypoint(RFWaypoint p_waypoint){
        rfTrajectory.addSegment(new RFSegment(p_waypoint, tangentOffset, constantHeading));
    }

    public void changeEndpoint(RFWaypoint p_endpoint){
        rfTrajectory.changeEndpoint(p_endpoint);
    }

    public void eraseWaypoints(){
        rfTrajectory.clear();
    }

    public boolean isFollowing(){
        return rfTrajectory.length()!=0;
    }

}
