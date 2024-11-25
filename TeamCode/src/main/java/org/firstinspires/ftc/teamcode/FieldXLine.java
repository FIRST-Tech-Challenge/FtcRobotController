package org.firstinspires.ftc.teamcode;

//this is a line that is parallel with the x-axis
//doesn't matter if x0 or x1 is larger
public class FieldXLine {
    final double blueX0, blueX1, blueY;
    final double redX0, redX1, redY;


    public FieldXLine(double blueX0, double blueX1, double blueY) {
        this.blueX0 = blueX0;
        this.blueX1 = blueX1;
        this.blueY = blueY;
        this.redX0 = -blueX0;
        this.redX1 = -blueX1;
        this.redY = -blueY;
    }

    public boolean isPointBetweenXsBlue(Point2d other) {
        return (other.x <= blueX0 && other.x >= blueX1) ||
               (other.x >= blueX0 && other.x <= blueX1);
    }
    public boolean isPointBetweenXsRed(Point2d other) {
        return  (other.x <= redX0 && other.x >= redX1) ||
                (other.x >= redX0 && other.x <= redX1);
    }

    public boolean isPointOnLineBlue(Point2d other, double allowedError) {
        return isPointBetweenXsBlue(other) && RobotMath.isAbsDiffWithinRange(blueY, other.y, allowedError);
    }

    public boolean isPointOnLineRed(Point2d other, double allowedError) {
        return isPointBetweenXsRed(other) && RobotMath.isAbsDiffWithinRange(redY, other.y, allowedError);
    }
}
