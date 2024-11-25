package org.firstinspires.ftc.teamcode;

//this is a line that is parallel with the y-axis
//doesn't matter if y0 or y1 is larger
public class FieldYLine {
    final double blueX, blueY0, blueY1;
    final double redX, redY0, redY1;


    public FieldYLine(double blueX, double blueY0, double blueY1) {
        this.blueX = blueX;
        this.blueY0 = blueY0;
        this.blueY1 = blueY1;
        this.redX = -blueX;
        this.redY0 = -blueY0;
        this.redY1 = -blueY1;
    }

    public boolean isPointBetweenYsBlue(Point2d other) {
        return (other.y <= blueY0 && other.y >= blueY1) ||
               (other.y >= blueY0 && other.y <= blueY1);
    }
    public boolean isPointBetweenYsRed(Point2d other) {
        return  (other.y <= redY0 && other.y >= redY1) ||
                (other.y >= redY0 && other.y <= redY1);
    }

    public boolean isPointOnLineBlue(Point2d other, double allowedError) {
        return isPointBetweenYsBlue(other) && RobotMath.isAbsDiffWithinRange(blueX, other.x, allowedError);
    }

    public boolean isPointOnLineRed(Point2d other, double allowedError) {
        return isPointBetweenYsRed(other) && RobotMath.isAbsDiffWithinRange(redX, other.x, allowedError);
    }
}
