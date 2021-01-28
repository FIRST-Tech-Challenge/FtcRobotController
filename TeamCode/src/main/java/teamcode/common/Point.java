package teamcode.common;

public class Point {
    public double x;
    public double y;

    private final double DEFAULT_VALUE_X = 0;
    private final double DEFAULT_VALUE_Y = 0;

    public Point(double x, double y){
        this.x = x;
        this.y = y;
    }

    public Point(){
        this.x = DEFAULT_VALUE_X;
        this.y = DEFAULT_VALUE_Y;
    }


    public void dilate(double scale) {
        this.x *= scale;
        this.y *= scale;
    }

    /**
     * calculate the slope
     * @param p2 the second point needed to form a line
     * @return the slope as a double
     */
    public double slope(Point p2){
        if((x - p2.x == 0)){
            return (y - p2.y) / 0.0001;
        }
        return (y - p2.y) / (x - p2.x);
    }

    public boolean equals(Point other){
        return this.y == other.y && this.x == other.x;
    }

    public String toString(){
        return "(" + this.x + "," + this.y + ")";
    }

    public String toStringData(){
        return this.x + " " + this.y;
    }

    public double getDistance(Point other){
        return Math.sqrt(Math.pow(this.y - other.y, 2) +   Math.pow(this.x - other.x, 2));
    }







}
