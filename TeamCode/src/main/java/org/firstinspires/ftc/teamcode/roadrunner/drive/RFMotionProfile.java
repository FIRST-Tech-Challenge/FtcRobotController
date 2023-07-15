package org.firstinspires.ftc.teamcode.roadrunner.drive;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentVelocity;

import static java.lang.Double.max;
import static java.lang.Double.min;
import static java.lang.Math.abs;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.ArrayList;

public class RFMotionProfile {
    double curviness, c, velo1, velo2, endVel, startVel;
    ArrayList<Double> tList;
    double MIN_DECEL = -5, startDist, endDist, totalDist, length, targetVelocity, targetAcceleration;
    int SEARCH_RESOLUTION = 12;

    public RFMotionProfile(Vector2d p_startVel, Vector2d p_endVel, double p_curviness) {
        curviness = min(p_curviness, 1);
        //convert from percentage to slope
        c = 2 * MAX_ACCEL * MAX_ACCEL * (1 - curviness) / (curviness * MAX_VEL);
        velo1 = MAX_ACCEL * MAX_ACCEL / (2 * c * c * c);
        velo2 = MAX_VEL - velo1;
        endVel = p_endVel.norm();
        startVel = p_startVel.norm();
    }

    public void setLength(double p_length) {
        if (length != p_length) {
            length = p_length;
            calculateTList(length);
        }
    }

    public double getInstantaneousTargetAcceleration(double dist) {
        double velo = currentVelocity.vec().norm(), targetAccel;
        if (velo < velo1) {
            targetAccel = c * Math.sqrt(2 * (velo) * c);
        } else if (velo < velo2) {
            targetAccel = MAX_ACCEL;
        } else if (velo > velo2 && velo < MAX_VEL) {
            targetAccel = MAX_ACCEL - c * Math.sqrt(2 * (velo - velo2));
        } else {
            targetAccel = 0;
        }
        //if u should decel
        if (dist < getAccelDist(currentVelocity.vec().norm(), endVel)) {
            targetAccel = min(targetAccel * -1, MIN_DECEL);
        } else {
            //nothing
        }
        return targetAccel;
    }

    public void calculateTList(double length) {
        tList = new ArrayList();
        double endT;
        if (startVel < velo1) {
            double t = 2 * c * sqrt(startVel);
            tList.add(time - t);
            startDist = c * t * t * t / 6;
        } else if (startVel < velo2) {
            double t1 = 2 * c * sqrt(velo1);
            double t2 = (startVel - velo1) / MAX_ACCEL;
            tList.add(time - t1 - t2);
            startDist = c * t1 * t1 * t1 / 6 + MAX_ACCEL * t2 * t2 / 2;
        } else {
            double t1 = 2 * c * sqrt(velo1);
            double t2 = (velo2 - velo1) / MAX_ACCEL;
            double t3 = 2 * c * sqrt(startVel - velo2);
            tList.add(time - t1 - t2 - t3);
            startDist = c * t1 * t1 * t1 / 6 + MAX_ACCEL * t2 * t2 / 2 + MAX_ACCEL * t3 * t3 / 2 - c * t3 * t3 * t3 / 6;
        }
        if (endVel < velo1) {
            endT = 2 * c * sqrt(endVel);
            endDist = c * endT * endT * endT / 6;
        } else if (endVel < velo2) {
            double t1 = 2 * c * sqrt(velo1);
            double t2 = (endVel - velo1) / MAX_ACCEL;
            endT = t1 + t2;
            endDist = c * t1 * t1 * t1 / 6 + MAX_ACCEL * t2 * t2 / 2;

        } else {
            double t1 = 2 * c * sqrt(velo1);
            double t2 = (velo2 - velo1) / MAX_ACCEL;
            double t3 = 2 * c * sqrt(endVel - velo2);
            endT = t1 + t2 + t3;
            endDist = c * t1 * t1 * t1 / 6 + MAX_ACCEL * t2 * t2 / 2 + MAX_ACCEL * t3 * t3 / 2 - c * t3 * t3 * t3 / 6;

        }
        totalDist = length + startDist + endDist;
        double peakVel = min(sqrt(totalDist * MAX_ACCEL * (1 - curviness * 0.5) / 2), MAX_VEL);
        //peakVel * time(peakVel) = rlDist
        //time(peakVel) = 2*peakVel/(1-curviness/2)/MAX_ACCEL
        double cruiseLength = max(length - getAccelDist(peakVel, endVel) - getAccelDist(startVel, peakVel), 0);
        tList.add(tList.get(0) + 2 * c * sqrt(min(velo1, peakVel)));
        tList.add(tList.get(1) + (min(velo2, peakVel) - min(velo1, peakVel)) / MAX_ACCEL);
        tList.add(tList.get(2) + 2 * c * sqrt(peakVel - min(velo2, peakVel)));
        tList.add(tList.get(3) + cruiseLength / peakVel);
        tList.add(tList.get(4) + 2 * c * sqrt(peakVel - min(velo2, peakVel)));
        tList.add(tList.get(5) + (min(velo2, peakVel) - min(velo1, peakVel)) / MAX_ACCEL);
        tList.add(tList.get(6) + 2 * c * sqrt(min(velo1, peakVel)));
        tList.add(endT);
    }

    public double calculateTargetVelocity(double time) {
        if (time > tList.get(7) - tList.get(8)) {
            time = tList.get(7) - tList.get(8);
        }
        if (time < tList.get(1)) {
            targetVelocity = startVel + 0.5 * c * (time - tList.get(0)) * (time - tList.get(0));
            return targetVelocity;
        } else if (time < tList.get(2)) {
            targetVelocity = velo1 + MAX_ACCEL * (time - tList.get(1));
            return targetVelocity;
        } else if (time < tList.get(3)) {
            targetVelocity = MAX_VEL - 0.5 * c * (tList.get(3) - time) * (tList.get(3) - time);
            return targetVelocity;
        } else if (time < tList.get(4)) {
            targetVelocity = MAX_VEL;
            return targetVelocity;
        } else if (time < tList.get(5)) {
            return MAX_VEL - 0.5 * c * (time - tList.get(4)) * (time - tList.get(4));
        } else if (time < tList.get(6)) {
            return velo2 - MAX_ACCEL * (time - tList.get(5));
        } else {
            if (time > tList.get(7)) {
                time = tList.get(7);
            }
            return velo1 - 0.5 * c * (time - tList.get(6)) * (time - tList.get(6));
        }
    }

    public double calculateTargetAcceleration(double time) {
        if (time > tList.get(7) - tList.get(8)) {
            time = tList.get(7) - tList.get(8);
        }
        if (time < tList.get(1)) {
            return c * (time - tList.get(0));
        } else if (time < tList.get(2)) {
            return MAX_ACCEL;
        } else if (time < tList.get(3)) {
            return MAX_ACCEL - c * (time - tList.get(2));
        } else if (time < tList.get(4)) {
            return 0;
        } else if (time < tList.get(5)) {
            return -c * (time - tList.get(4));
        } else if (time < tList.get(6)) {
            return -MAX_ACCEL;
        } else {
            if (time > tList.get(7)) {
                time = tList.get(7);
            }
            return -MAX_ACCEL + c * (time - tList.get(6));
        }
    }

    //input current time, output amount of distance traveled
    public double motionProfileTimeToDist(double time) {
        if (time > tList.get(7) - tList.get(8)) {
            time = tList.get(7) - tList.get(8);
        }
        if (time > tList.get(6)) {
            double t = tList.get(7) - time;
            return totalDist - c * t * t * t / 6;
        } else if (time > tList.get(5)) {
            double t1 = tList.get(7) - tList.get(6);
            double t2 = tList.get(6) - time;
            return totalDist - (c * t1 * t1 * t1 / 6 + velo1 * t2 + MAX_ACCEL * t2 * t2 * 0.5);
        } else if (time > tList.get(4)) {
            double t1 = tList.get(7) - tList.get(6);
            double t2 = tList.get(6) - tList.get(5);
            double t3 = tList.get(5) - time;
            return totalDist - (c * t1 * t1 * t1 / 6 + velo1 * t2 + MAX_ACCEL * t2 * t2 * 0.5 + velo2 * t3 + MAX_ACCEL * t3 * t3 * 0.5 - c * t3 * t3 * t3 / 6);
        } else if (time > tList.get(3)) {
            double t1 = tList.get(7) - tList.get(6);
            double t2 = tList.get(6) - tList.get(5);
            double t3 = tList.get(5) - tList.get(4);
            double t4 = tList.get(4) - time;
            return totalDist - (c * t1 * t1 * t1 / 6 + velo1 * t2 + MAX_ACCEL * t2 * t2 * 0.5 + velo2 * t3 + MAX_ACCEL * t3 * t3 * 0.5 - c * t3 * t3 * t3 / 6 +
                    MAX_VEL * t4);
        } else if (time > tList.get(2)) {
            double t1 = tList.get(7) - tList.get(6);
            double t2 = tList.get(6) - tList.get(5);
            double t3 = tList.get(5) - tList.get(4);
            double t4 = tList.get(4) - tList.get(3);
            double t5 = tList.get(3) - time;
            return totalDist - (c * t1 * t1 * t1 / 6 + velo1 * t2 + MAX_ACCEL * t2 * t2 * 0.5 + velo2 * t3 + MAX_ACCEL * t3 * t3 * 0.5 - c * t3 * t3 * t3 / 6 +
                    MAX_VEL * t4 + MAX_VEL * t5 - c * t5 * t5 * t5 / 6);
        } else if (time > tList.get(1)) {
            double t1 = tList.get(7) - tList.get(6);
            double t2 = tList.get(6) - tList.get(5);
            double t3 = tList.get(5) - tList.get(4);
            double t4 = tList.get(4) - tList.get(3);
            double t5 = tList.get(3) - tList.get(2);
            double t6 = tList.get(2) - time;
            return totalDist - (c * t1 * t1 * t1 / 6 + velo1 * t2 + MAX_ACCEL * t2 * t2 * 0.5 + velo2 * t3 + MAX_ACCEL * t3 * t3 * 0.5 - c * t3 * t3 * t3 / 6 +
                    MAX_VEL * t4 + MAX_VEL * t5 - c * t5 * t5 * t5 / 6 + velo2 * t6 - MAX_ACCEL * t6 * t6 * 0.5);
        } else {
            if (time < tList.get(0)) {
                time = tList.get(0);
            }
            double t1 = tList.get(7) - tList.get(6);
            double t2 = tList.get(6) - tList.get(5);
            double t3 = tList.get(5) - tList.get(4);
            double t4 = tList.get(4) - tList.get(3);
            double t5 = tList.get(3) - tList.get(2);
            double t6 = tList.get(2) - tList.get(1);
            double t7 = tList.get(1) - time;
            return totalDist - (c * t1 * t1 * t1 / 6 + velo1 * t2 + MAX_ACCEL * t2 * t2 * 0.5 + velo2 * t3 + MAX_ACCEL * t3 * t3 * 0.5 - c * t3 * t3 * t3 / 6 +
                    MAX_VEL * t4 + MAX_VEL * t5 - c * t5 * t5 * t5 / 6 + velo2 * t6 - MAX_ACCEL * t6 * t6 * 0.5 + velo1 * t7 - MAX_ACCEL * t7 * t7 * 0.5
                    + c * t7 * t7 * t7 / 6);
        }
    }

    //take in remaining dist, convert to remaining time
    public double motionProfileRemDistToRemTime(double dist) {
        dist = totalDist - dist - endDist;
        double time = 0.5;
        for (int i = 0; i < SEARCH_RESOLUTION - 1; i++) {
            double searchDist = motionProfileTimeToDist(tList.get(7) * (time) + tList.get(0) * (1 - time));
            double error = dist - searchDist;
            if (abs(error) > 0.01) {
                if (error > 0) {
                    time += (1 * pow(2, -i - 2));
                } else {
                    time -= (1 * pow(2, -i - 2));
                }
            }
        }
        return tList.get(7) * (time) + tList.get(0) * (1 - time) - tList.get(8);
    }

    public double getDecelTime(double vel) {
        return (vel - endVel) / ((1 - curviness * 0.5) * MAX_ACCEL);
    }

    public double getDecelDist(double vel) {
        return getDecelTime(vel) * (vel + endVel) * 0.5;
    }

    public double getAccelTime(double vel) {
        return (MAX_VEL - vel) / ((1 - curviness * 0.5) * MAX_ACCEL);
    }

    public double getAccelDist(double vel) {
        return getAccelTime(vel) * (vel + endVel) * 0.5;
    }

    public double getAccelTime(double vel1, double vel2) {
        return abs(vel1 - vel2) / ((1 - curviness * 0.5) * MAX_ACCEL);
    }

    public double getAccelDist(double vel1, double vel2) {
        return getAccelTime(vel1, vel2) * (vel1 + vel2) * 0.5;
    }
}
