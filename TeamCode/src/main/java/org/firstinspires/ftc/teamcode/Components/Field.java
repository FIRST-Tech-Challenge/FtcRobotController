package org.firstinspires.ftc.teamcode.Components;

import static java.lang.Math.PI;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;

import org.firstinspires.ftc.teamcode.Components.CV.CVMaster;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class Field {
    private SampleMecanumDrive roadrun;
    private CVMaster cv;
    double lookingDistance = 20.0, dropDistance = 9;
    double[] poleValues = {10, 10, 0, 0}, cameraPos = {4, 8, 0}, dropPosition = {0, 0, 0};


    double[][][] poleCoords = {
            //row 1
            {{-47, 47}, {-47, 23.5}, {-47, 0}, {-47, -23.5}, {-47, -47}},
            //row 2
            {{-23.5, 47}, {-23.5, 23.5}, {-23.5, 0}, {-23.5, -23.5}, {-23.5, -47}},
            //row 3
            {{0, 47}, {0, 23.5}, {0, 0}, {0, -23.5}, {0, -47}},
            //row 4
            {{23.5, 47}, {23.5, 23.5}, {23.5, 0}, {23.5, -23.5}, {23.5, -47}},
            //row 5
            {{47, 47}, {47, 23.5}, {47.5, 0}, {47, -23.5}, {47, -47}}};

    double[][][] tileCoords = {
            //row 1
            {{-58.75, 58.75}, {-58.75, 35.25}, {-58.75, 11.75}, {-58.75, -11.75}, {-58.75, -35.25}, {-58.75, -58.75}},
            //row 2
            {{-35.25, 58.75}, {-35.25, 35.25}, {-35.25, 11.75}, {-35.25, -11.75}, {-35.25, -35.25}, {-58.75, -58.75}},
            //row 3
            {{-11.75, 58.75}, {11.75, 35.25}, {11.75, 11.75}, {11.75, -11.75}, {11.75, -35.25}, {-58.75, -58.75}},
            //row 4
            {{11.75, 58.75}, {11.75, 35.25}, {11.75, 11.75}, {11.75, -11.75}, {11.75, -35.25}, {-58.75, -58.75}},
            //row 5
            {{35.25, 58.75}, {35.25, 35.25}, {35.25, 11.75}, {35.25, -11.75}, {35.25, -35.25}, {-58.75, -58.75}},
            //row 6
            {{58.75, 58.75}, {58.75, 35.25}, {58.75, 11.75}, {58.75, -11.75}, {58.75, -35.25}, {-58.75, -58.75}}};

    public Field(SampleMecanumDrive p_roadrun, CVMaster p_cv) {
        roadrun = p_roadrun;
        cv = p_cv;
    }

    //which tile bot at
    public double[][][] atTile() {
        // first set of coords is for which tile e.g. {A,1}(0,0), second is for {x,y} in inches


        return tileCoords;
    }

    //is robot looking at a pole
    public boolean lookingAtPole() {
        double[] minDistPole = minDistPole();
        if (minDistPole[0] != 10) {
            double[] coords = null/*cv.rotatedPolarCoordDelta()*/;
            dropPosition[0] = roadrun.getPoseEstimate().getX() + (coords[1] - dropDistance) * cos(roadrun.getPoseEstimate().getHeading() + coords[0]);
            dropPosition[1] = roadrun.getPoseEstimate().getY() + (coords[1] - dropDistance) * sin(roadrun.getPoseEstimate().getHeading() + coords[0]);
            dropPosition[2] = roadrun.getPoseEstimate().getHeading() + coords[0];
//            roadrun.setPoseEstimate(new Pose2d(2 * roadrun.getPoseEstimate().getX() + (coords[1]) * cos(roadrun.getPoseEstimate().getHeading() + coords[0]) - poleCoords[(int) poleValues[0]][(int) poleValues[1]][0],
//                    2 * roadrun.getPoseEstimate().getY() + (coords[1]) * sin(roadrun.getPoseEstimate().getHeading() + coords[0]) - poleCoords[(int) poleValues[0]][(int) poleValues[1]][1],
//                    2 * roadrun.getPoseEstimate().getHeading() + coords[0] - poleValues[3]));
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
        dist[0] += sin(roadrun.getPoseEstimate().getHeading() + cameraPos[2]) * cameraPos[0] + cos(roadrun.getPoseEstimate().getHeading() + cameraPos[2]) * cameraPos[1];
        dist[1] += cos(roadrun.getPoseEstimate().getHeading() + cameraPos[2]) * cameraPos[0] + sin(roadrun.getPoseEstimate().getHeading() + cameraPos[2]) * cameraPos[1];
        double[] polarCoords = toPolar(dist);
        if (polarCoords[0] < lookingDistance && ((polarCoords[1] - roadrun.getPoseEstimate().getHeading()) * 180.0 / PI + 180) % 360 < 22.5) {
            poleValues = new double[]{p_i, p_j, polarCoords[0], polarCoords[1] - roadrun.getPoseEstimate().getHeading() + PI};
            return true;
        } else {
            return false;
        }
    }
}
