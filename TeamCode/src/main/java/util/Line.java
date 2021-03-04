package util;

public class Line {
    public double x1;
    public double y1;
    public double x2;
    public double y2;
    public double mx;
    public double my;

    public Line(double x1, double y1, double x2, double y2){
        this.x1 = x1;
        this.y1 = y1;
        this.x2 = x2;
        this.y2 = y2;

        mx = x2-x1;
        my = y2-y1;

    }

    public double[] getAt(double t){
        return new double[]{(x1)+(mx*t),(y1)+(my*t) };
    }

    public double getDis(){
        double x = x2-x1;
        double y = y2-y1;
        return Math.sqrt(x*x+y*y);
    }

}
