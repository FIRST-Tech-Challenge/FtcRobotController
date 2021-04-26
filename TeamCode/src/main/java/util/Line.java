package util;

public class Line {
    //start coords (x1,y1) end coords (x2,y2) mx is slope in x dir from 0-1 and my is the same for y
    public double x1;
    public double y1;
    public double x2;
    public double y2;
    public double mx;
    public double my;

    //Define line using endpoints
    public Line(double x1, double y1, double x2, double y2){
        this.x1 = x1;
        this.y1 = y1;
        this.x2 = x2;
        this.y2 = y2;

        mx = x2-x1;
        my = y2-y1;

    }
    //Gets the position of the line at a certain t value
    public double[] getAt(double t){
        return new double[]{(x1)+(mx*t),(y1)+(my*t) };
    }
    //Gets the length of the line
    public double getDis(){
        double x = x2-x1;
        double y = y2-y1;
        return Math.sqrt(x*x+y*y);
    }

}
