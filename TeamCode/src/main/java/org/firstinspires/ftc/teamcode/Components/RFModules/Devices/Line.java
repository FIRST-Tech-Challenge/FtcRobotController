package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static java.lang.Math.pow;
import static java.lang.Math.signum;
import static java.lang.Math.sqrt;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class Line {
    double a, b, c;
    Vector2d range1;
    Vector2d range2;
    boolean positiveSlope;
    public Line(double p_a, double p_b, double p_c, Vector2d p_range1, Vector2d p_range2){
        a = p_a;
        b = p_b;
        c = p_c;
        range1 = p_range1;
        range2 = p_range2;

        positiveSlope = (!(a > 0) || !(b > 0)) && (!(a < 0) || !(b < 0));
    }
    public double distToLine(){
        double[] intercept = findIntercept();
        return sqrt(pow(intercept[0] - currentPose.getX(), 2) + pow(intercept[1] - currentPose.getY(), 2));
    }

    public double[] findIntercept() {
        /*normal
        ax + by = c
        by = -ax +c
        ay = bx + c
        x = (ay-c)/b

        perp
        y =b/a x -b/a * pose x + pose y
        (c/b - c1)/(b/a + a/b)
        * */
        double perp_c =- b * currentPose.getX() + a * currentPose.getY();
        double int_x =0,int_y=0;
        if(a!=0){
            int_x=c-perp_c*b/a;
            int_x/=b*b/a+a;
            int_y=b/a*int_x+perp_c/a;
        }
        else if(b!=0){
            int_x=(a*c/b-perp_c)/(a*a/b+b);
            int_y=-a/b*int_x+c/b;
        }
        packet.put("int_x", int_x );
        packet.put("int_y",  int_y);


        if (int_x < range1.getX()) {
            int_x = range1.getX();

            if (positiveSlope) int_y = range1.getY();
            else int_y = range2.getY();
        }
        else if (int_x > range2.getX()) {
            int_x = range2.getX();

            if (positiveSlope) int_y = range2.getY();
            else int_y = range1.getY();
        }

        if (int_y < range1.getY()) {
            int_x = range1.getX();
            int_y = range1.getY();

        }
        else if (int_y > range2.getY()) {
            int_x = range1.getX();
            int_y = range2.getY();
        }
        packet.put("int_x1", int_x );
        packet.put("int_y1",  int_y);

        return new double[]{int_x, int_y};
    }
}
