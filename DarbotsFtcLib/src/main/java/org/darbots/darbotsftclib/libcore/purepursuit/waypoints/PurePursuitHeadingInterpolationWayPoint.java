package org.darbots.darbotsftclib.libcore.purepursuit.waypoints;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;

public class PurePursuitHeadingInterpolationWayPoint extends PurePursuitWayPoint {
    private double m_Heading;
    private double m_AllowedHeadingError = 5;
    public double getDesiredHeading(){
        return this.m_Heading;
    }
    public void setDesiredHeading(double desiredHeading){
        this.m_Heading = XYPlaneCalculations.normalizeDeg(m_Heading);
    }
    public double getAllowedHeadingError(){
        return this.m_AllowedHeadingError;
    }
    public void setAllowedHeadingError(double allowedHeadingError){
        this.m_AllowedHeadingError = Math.abs(allowedHeadingError);
    }
    public PurePursuitHeadingInterpolationWayPoint(double X, double Y, double Heading){
        super(X,Y);
        this.setDesiredHeading(Heading);
    }
    public PurePursuitHeadingInterpolationWayPoint(PurePursuitHeadingInterpolationWayPoint point){
        super(point);
        this.m_Heading = point.m_Heading;
        this.m_AllowedHeadingError = point.m_AllowedHeadingError;
    }
    public PurePursuitHeadingInterpolationWayPoint(PurePursuitWayPoint point, double Heading){
        super(point);
        this.setDesiredHeading(Heading);
    }
    public PurePursuitHeadingInterpolationWayPoint(RobotPoint2D point, double Heading){
        super(point);
        this.setDesiredHeading(Heading);
    }
}
