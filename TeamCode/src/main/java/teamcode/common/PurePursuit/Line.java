package teamcode.common.PurePursuit;

import teamcode.common.Point;

public class Line {
    Point p1;
    Point p2;
    Double slope;
    //note: if the slopes value is equal to null, then that means that the slope is vertical

    /**
     *
     * @param p1 the point leftward on the x axis
     * @param p2 the point rightward on rhe x axis
     */
    public Line(Point p1, Point p2){
        this.p1 = p1;
        this.p2 = p2;
        slope = p2.slope(p1);

    }

    /**
     *
     * @return returns the y intercept of the line, note that this only functions if you have the line being interpreted left to right
     */
    public double getyIntercept(){
        return p1.y - (slope * p1.x);
    }

    /**
     *
     * @return the angle that is created by the line in Radians
     */
    public double getAngleRads(){
        return Math.atan2(Math.abs(p1.y - p2.y),  Math.abs(p1.x - p2.x));
    }


    /**
     * @param x an x value within the domain of the line
     * @return a y value on the line given an x
     */
    public double getValue(double x){
        return x * slope + getyIntercept();
    }


    /**
     * method used to solve for the intersection between 2 lines
     * @param other the other line in the system of equations
     * @return the point of intersection between the 2 lines
     */
    public Point intersection(Line other) {
        double xValue = (other.getyIntercept() - getyIntercept()) / (slope - other.slope);
        assert getValue(xValue) == other.getValue(xValue);
        return new Point(xValue, getValue(xValue));

    }

    /**
     * method used to solve for the intersection between 2 line segments
     * @param other the other line in the system of equations
     * @return the point of intersection between the 2 lines, if the intersection is out of the domain of the
     * line segment, the method will return null
     */
    public Point segmentIntersection( Line other){
        double xValue = 0;
        if(slope != null && other.slope != null) {
            xValue = (other.getyIntercept() - getyIntercept()) / (slope - other.slope);
        }else if(slope == null){
            xValue = p1.x;
        }else if(other.slope == null){
            xValue = other.p1.x;
        }
        if(p1.x < xValue && p2.x > xValue){
            return new Point(xValue, getValue(xValue));
        }else{
            return null;
        }

    }

    public String toString(){
        if(slope != null){
            return "y = " + slope + "x + " + getyIntercept();
        } else{
            return "x = " + p1.x;
        }
    }
}
