package org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Localizer;

import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

public class Spline {
    public double point[][] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};

    public Spline(double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3) {
        point[0][0] = x0;
        point[0][1] = y0;
        point[1][0] = x1;
        point[1][1] = y1;
        point[2][0] = x2;
        point[2][1] = y2;
        point[3][0] = x3;
        point[3][1] = y3;
    }

    public double[] getTarget(double[] pos, double velocity) {
        double[] target = {0, 0, 0};
        double twoDistance = sqrt(pow(point[2][1] - pos[1], 2) + pow(point[2][0] - pos[0], 2));
        double oneDistance = sqrt(pow(point[1][1] - pos[1], 2) + pow(point[1][0] - pos[0], 2));
        double t = 0;
        if ((oneDistance + velocity / 3) / (oneDistance + twoDistance) > t) {
            t = (oneDistance + velocity / 3) / (oneDistance + twoDistance);
        }
        if (t > 1) {
            t = 1;
        }
        target[0] = 0.5 * ((2 * point[1][0]) + (-point[0][0] + point[2][0]) * t + (2 * point[0][0] - 5 * point[1][0] + 4
                * point[2][0] - point[3][0]) * pow(t, 2) + (-point[0][0] + 3 * point[1][0] - 3 * point[2][0] + point[3][0])
                * pow(t, 3));

        target[1] = 0.5 * ((2 * point[1][1]) + (-point[0][1] + point[2][1]) * t + (2 * point[0][1] - 5 * point[1][1] + 4 *
                point[2][1] - point[3][1]) * pow(t, 2) + (-point[0][1] + 3 * point[1][1] - 3 * point[2][1] + point[3][1]) * pow(t, 3));
        target[2] = (0.5 * ((point[2][1] - point[0][1]) + 2 * (2 * point[0][1] - 5 * point[1][1] + 4 * point[2][1] - point[3][1]) * t +
                3 * (-point[0][1] + 3 * point[1][1] - 3 * point[2][1] + point[3][1]) * pow(t, 2)));
        return target;
    }

    public double[] getError(double[] pos) {
        double[] error = {0, 0, 0};
        double[] target = {0, 0, 0};
        double twoDistance = sqrt(pow(point[2][1] - pos[1], 2) + pow(point[2][0] - pos[0], 2));
        double oneDistance = sqrt(pow(point[1][1] - pos[1], 2) + pow(point[1][0] - pos[0], 2));
        double t = (oneDistance) / (oneDistance + twoDistance);
        if (t > 1) {
            t = 1;
        }
        target[0] = 0.5 * ((2 * point[1][0]) + (-point[0][0] + point[2][0]) * t + (2 * point[0][0] - 5 * point[1][0] + 4
                * point[2][0] - point[3][0]) * pow(t, 2) + (-point[0][0] + 3 * point[1][0] - 3 * point[2][0] + point[3][0])
                * pow(t, 3));

        target[1] = 0.5 * ((2 * point[1][1]) + (-point[0][1] + point[2][1]) * t + (2 * point[0][1] - 5 * point[1][1] + 4 *
                point[2][1] - point[3][1]) * pow(t, 2) + (-point[0][1] + 3 * point[1][1] - 3 * point[2][1] + point[3][1]) * pow(t, 3));
        target[2] = (0.5 * ((point[2][1] - point[0][1]) + 2 * (2 * point[0][1] - 5 * point[1][1] + 4 * point[2][1] - point[3][1]) * t +
                3 * (-point[0][1] + 3 * point[1][1] - 3 * point[2][1] + point[3][1]) * pow(t, 2)));
        error[0] = target[0] - pos[0];
        error[1] = target[1] - pos[1];
        error[2] = target[2] - pos[2];
        return error;
    }
}