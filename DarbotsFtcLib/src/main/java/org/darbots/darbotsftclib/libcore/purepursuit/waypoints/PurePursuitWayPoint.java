package org.darbots.darbotsftclib.libcore.purepursuit.waypoints;

import com.qualcomm.robotcore.util.Range;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore.templates.DarbotsAction;

public class PurePursuitWayPoint extends RobotPoint2D {
    private double m_FollowDistance = 20;
    private double m_EndFollowNormalizedSpeed = 0;
    public DarbotsAction SegmentBeginAction = null;
    public boolean skipActionWhenSegmentFinished = false;
    public boolean stopActionWhenSkipping = true;
    public double allowedSecondsForThisSegment = -1;

    public double getEndFollowNormalizedSpeed(){
        return this.m_EndFollowNormalizedSpeed;
    }

    public void setEndFollowNormalizedSpeed(double endFollowNormalizedSpeed){
        this.m_EndFollowNormalizedSpeed = Range.clip(Math.abs(endFollowNormalizedSpeed),0,1.0);
    }

    public double getFollowDistance(){
        return this.m_FollowDistance;
    }
    public void setFollowDistance(double followDistance){
        this.m_FollowDistance = Math.abs(followDistance);
    }
    public PurePursuitWayPoint(double X, double Y){
        super(X,Y);
    }
    public PurePursuitWayPoint(double X, double Y, double FollowRadius, double endFollowNormalizedSpeed){
        super(X,Y);
        this.setFollowDistance(FollowRadius);
        this.setEndFollowNormalizedSpeed(endFollowNormalizedSpeed);
    }
    public PurePursuitWayPoint(RobotPoint2D point){
        super(point);
    }
    public PurePursuitWayPoint(PurePursuitWayPoint point){
        super(point);
        this.m_FollowDistance = point.m_FollowDistance;
        this.m_EndFollowNormalizedSpeed = point.m_EndFollowNormalizedSpeed;
        this.SegmentBeginAction = point.SegmentBeginAction;
        this.stopActionWhenSkipping = point.stopActionWhenSkipping;
        this.skipActionWhenSegmentFinished = point.skipActionWhenSegmentFinished;
        this.allowedSecondsForThisSegment = point.allowedSecondsForThisSegment;
    }
}
