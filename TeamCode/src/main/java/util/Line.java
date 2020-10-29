package util;

public class Line {
    public double x1 = 0;
    public double y1 = 0;
    public double x2 = 0;
    public double y2 = 0;
    public double mx = 0;
    public double my = 0;

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

}
