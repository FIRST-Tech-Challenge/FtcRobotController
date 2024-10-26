package org.firstinspires.ftc.teamcode.Drivetrain.Geometry;

import org.firstinspires.ftc.teamcode.Utils.MotionProfile;

public class Path {
    public double[][] waypoints;
    public boolean useStaticHeading;
    public double finalHeading;
    public MotionProfile motionProfile;
    public boolean reverse;
    // Maybe a private double[] finalPoint?

    // I'm not sure why you change the parameter argument names. Just keep it the same.
    // wp should be waypoints, mp should be motionProfile, finalDeg should be finalHeading,
    // isReversed should be reversed. staticHeading should be useStaticHeading.
    // to make it more readaable just do things like:
    // this.waypoints = waypoints.

    // Side note: I think the CLEARNER way would be to keep these private and make getters and setters for them.
    // In fact, this should be done with most things but I've been lenient with you on it. However I think
    // Path would be a great example of a class where getters and setters would amek things clean.
    // so for example, you'd have private double[][] waypoints, and a getWaypoints() function. Instead of
    // path.waypoints, you'd call path.getWaypoints()!
    public Path(double[][] waypoints, MotionProfile motionProfile, double finalHeading, boolean reverse, boolean useStaticHeading){
        this.waypoints = waypoints;
        this.motionProfile = motionProfile;
        this.finalHeading = finalHeading;
        this.reverse = reverse;
        this.useStaticHeading = useStaticHeading;

        // Also, maybe to make it easy to reach, grab the final waypoint and actually set it here.
        // just grab the last element of waypoints and set it here!
    }
    public MotionProfile getMotionProfile(){
        return motionProfile;
    }
    public double[][] getWaypoints(){
        return waypoints;
    }

}