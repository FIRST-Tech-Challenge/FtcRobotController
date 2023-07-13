package org.firstinspires.ftc.teamcode.roadrunner.drive;

import static java.lang.Double.max;
import static java.lang.Double.min;

public class RFSegment {
    private RFWaypoint waypoint;
    private double tangentOffset;
    private double curviness;
    private boolean constantHeading;
    private boolean isCompiled;

    public RFSegment(RFWaypoint p_waypoint, double p_tangentOffset, boolean p_constantHeading, double curviness){
        createRFSegment(p_waypoint,p_tangentOffset,p_constantHeading,curviness);
    }
    public RFSegment(RFWaypoint p_waypoint, double p_tangentOffset, boolean p_constantHeading){
        createRFSegment(p_waypoint,p_tangentOffset,p_constantHeading,0);
    }

    public void createRFSegment(RFWaypoint p_waypoint, double p_tangentOffset, boolean p_constantHeading, double curviness){
        waypoint = p_waypoint;
        tangentOffset = p_tangentOffset;
        constantHeading = p_constantHeading;
        curviness = max(min(1,curviness),0);
        isCompiled = false;
    }

    public RFWaypoint getWaypoint() {
        return waypoint;
    }

    public double getTangentOffset() {
        return tangentOffset;
    }

    public boolean isConstantHeading() {
        return constantHeading;
    }

    public boolean isCompiled() {
        return isCompiled;
    }

    public double getCurviness() {
        return curviness;
    }

    public void setCompiled(boolean p_compiled){
        isCompiled = p_compiled;
    }
}
