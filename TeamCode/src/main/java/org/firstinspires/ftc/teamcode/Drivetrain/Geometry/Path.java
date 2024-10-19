package org.firstinspires.ftc.teamcode.Drivetrain.Geometry;

import org.firstinspires.ftc.teamcode.Utils.MotionProfile;

public class Path {
    public double[][] waypoints;
    public boolean useStaticHeading;
    public double finalHeading;
    public MotionProfile motionProfile;
    public boolean reverse;

    public Path(double[][] wp, MotionProfile mp, double finalDeg, boolean isReversed, boolean staticHeading){
        waypoints = wp;
        motionProfile = mp;
        finalHeading = finalDeg;
        reverse = isReversed;
        useStaticHeading = staticHeading;
    }

}
