package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class Line {
    double a, b, c;
    Vector2d range1;
    Vector2d range2;
    Pose2d rangeStart;
    public Line(double p_a, double p_b, double p_c, Pose2d p_rangeStart, Vector2d p_range1, Vector2d p_range2){
        a = p_a;
        b = p_b;
        c = p_c;
        range1 = p_range1;
        range2 = p_range2;
        rangeStart = p_rangeStart;
    }
    public double distToLine(){
        double[] intercept = findIntercept();
        return sqrt(pow(intercept[0] - currentPose.getX(), 2) + pow(intercept[1] - currentPose.getY(), 2));
    }

    public double[] findIntercept() {
        double perp_c = b * currentPose.getX() - a * currentPose.getY();
        double int_x = (a * c + b * perp_c)/(pow(a, 2) + pow(b, 2));
        double int_y = (-a * int_x + c)/b;

        if (int_x <= rangeStart.getX()) {
            int_x = rangeStart.getX();
            int_y = (-a * int_x + c)/b;
        }
        else if (int_x >= rangeStart.getX() + range1.getX()) {
            int_x = rangeStart.getX() + range1.getX();
            int_y = (-a * int_x + c)/b;
        }

        if (int_y <= rangeStart.getY()) {
            int_y = rangeStart.getY();
            int_x = (-b * int_y + c)/a;
        }
        else if (int_y >= rangeStart.getY() + range2.getY()) {
            int_y = rangeStart.getY() + range2.getY();
            int_x = (-b * int_y + c)/a;
        }

        return new double[]{int_x, int_y};
    }
}
