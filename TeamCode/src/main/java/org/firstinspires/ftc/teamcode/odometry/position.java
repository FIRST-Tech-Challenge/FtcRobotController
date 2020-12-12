package org.firstinspires.ftc.teamcode.odometry;


public class position {
    static double ROBOTWIDTH;
    static double X = 0;
    static double Y = 0;

    public static void calculate(double INNERINCH, double OUTERINCH){

        double arc =  (INNERINCH*ROBOTWIDTH)/(OUTERINCH-INNERINCH);
        double radius = (arc*INNERINCH)/(arc-ROBOTWIDTH);

        //fact check this next line-using arc and radius to find angle
        //I wrote equation as angle = arc/radius and then convert to degrees
        double centralAngle =( arc/radius)*(180/Math.PI);
        double addY = Math.sin(centralAngle)*radius;
        double addX = radius - (Math.sqrt(radius*radius)-(addY*addY));

        X+=addX;
        Y+=addY;
    }
}
