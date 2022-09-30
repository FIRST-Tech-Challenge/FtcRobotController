package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class Field {
    private SampleMecanumDrive roadrun;
    double lookingDistance = 20.0;
    double[] poleValues = {10, 10, 0, 0}, cameraPos = {4, 8};
                              //row 1
    double[][][] poleCoords = {{{-47, 47}, {-47, 23.5}, {-47, 0}, {-47, -23.5}, {-47, -47}},
            //row 2
            {{-23.5, 47}, {-23.5, 23.5}, {-23.5, 0}, {-23.5, -23.5}, {-23.5, -47}},
            //row 3
            {{0, 47}, {0, 23.5}, {0, 0}, {0, -23.5}, {0, -47}},
            //row 4
            {{23.5, 47}, {23.5, 23.5}, {23.5, 0}, {23.5, -23.5}, {23.5, -47}},
            //row 5
            {{47, 47}, {47, 23.5}, {47.5, 0}, {47, -23.5}, {47, -47}}};

    public Field(SampleMecanumDrive p_roadrun) {
        roadrun = p_roadrun;
    }

    //which tile bot at
    public int[][] atTile() {
        // first set of coords is for which tile e.g. {A,1}(0,0), second is for {x,y} in inches
        int[][] tileCoords = {{0, 0}, {0, 0}};

        return tileCoords;
    }

    //is robot looking at a pole
    public boolean lookingAtPole() {
        double[] minDistPole = minDistPole();
        if (minDistPole[0] != 10) {
            return true;
        }
        return false;
    }

    //which pole bot looking at
    public double[] lookedAtPole() {
        // first two indexes of coords is for which tile e.g. {V,1}(0,0), second two idexes is for {r,theta} in inches/radians
        return poleValues;
    }

    // returns which tile is closest with first two coords, third index is distance in inches
    public double[] minDistTile() {
        double[] closestTile = {0, 0, 0};

        return closestTile;
    }

    // returns which pole is closest with first two indexes as pole coords, third index is distance in inches, fourth index is angle in radians
    public double[] minDistPole() {
        double[] closestPole = {10, 10, 0, 0};
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 5; j++) {
                if (i % 2 == 0 && j % 2 == 0) {
                    continue;
                }
                if (inCloseSight(i, j)) {
                    return poleValues;
                } else {
                    //do nothing
                }
            }
        }
        return closestPole;
    }

    public double[] toPolar(double[] p_coords) {
        return new double[]{pow(p_coords[0] * p_coords[0] + p_coords[1] * p_coords[1], .5), atan2(p_coords[1], p_coords[0])};
    }

    public boolean inCloseSight(int p_i, int p_j) {
        double[] dist = {roadrun.getPoseEstimate().getX() - poleCoords[p_i][p_j][0], roadrun.getPoseEstimate().getY() - poleCoords[p_i][p_j][1]};
        dist[0] += sin(roadrun.getPoseEstimate().getHeading()) * cameraPos[0] + cos(roadrun.getPoseEstimate().getHeading()) * cameraPos[1];
        dist[1] += cos(roadrun.getPoseEstimate().getHeading()) * cameraPos[0] + sin(roadrun.getPoseEstimate().getHeading()) * cameraPos[1];
        double[] polarCoords = toPolar(dist);
        if (p_i == 3 && p_j == 0) {
            op.telemetry.addData("dist", polarCoords[0]);
            op.telemetry.addData("theta", (abs(polarCoords[1] - roadrun.getPoseEstimate().getHeading()) * 180 / PI-180) % 360);
        }
        if (polarCoords[0] < lookingDistance && (abs(polarCoords[1] - roadrun.getPoseEstimate().getHeading()) * 180 / PI-180) % 360 < 22.5) {
            poleValues = new double[]{p_i, p_j, polarCoords[0], polarCoords[1]};
            return true;
        } else {
            return false;
        }
    }
}
