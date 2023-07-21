package org.firstinspires.ftc.teamcode.roadrunner.drive;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentVelocity;

import static java.lang.Double.POSITIVE_INFINITY;
import static java.lang.Double.max;
import static java.lang.Double.min;
import static java.lang.Math.abs;
import static java.lang.Math.cbrt;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.ArrayList;

public class RFMotionProfile {
    double curviness, c, velo1, velo2, endVel, startVel;
    ArrayList<Double> tList;
    double MIN_DECEL = -5, startDist, endDist, totalDist, length, targetVelocity, peakVel, targetAcceleration;
    int SEARCH_RESOLUTION = 12;

    public RFMotionProfile(Vector2d p_startVel, Vector2d p_endVel, double p_curviness) {
        curviness = min(p_curviness, 1);
        //convert from percentage to slope
        double t = MAX_VEL / (MAX_ACCEL * (1 - curviness * 0.5));
        packet.put("curviness", curviness);
        c = MAX_ACCEL / (t * curviness * 0.5);
        if (c == POSITIVE_INFINITY) {
            velo1 = 0;
            velo2 = MAX_VEL;
        } else {
            velo1 = MAX_ACCEL * 0.5 * (t * curviness * 0.5);
            velo2 = MAX_VEL - velo1;
        }
        endVel = p_endVel.norm();
        startVel = p_startVel.norm();
    }

    public void setLength(double p_length) {
        if (length != p_length) {
            length = p_length;
            calculateTList(length);
            packet.put("inpudDist1", length);
        }
    }

    public boolean isProfileDone(double time) {
        return time > tList.get(7) - tList.get(8);
    }

    public double getInstantaneousTargetAcceleration(double dist) {
        double velo = currentVelocity.vec().norm(), targetAccel;
        if (velo < velo1) {
            targetAccel = c * Math.sqrt(2 * (velo) * c);
        } else if (velo < velo2) {
            targetAccel = MAX_ACCEL;
        } else if (velo > velo2 && velo < peakVel) {
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

    //v=c, c =a, a =b, d=d
    public double solveCubicEquation(int A, int B, int C, int D) {

        double x1, x2 = 0, x3;
        double a = (double) B / A;
        double b = (double) C / A;
        double c = (double) D / A;

        double p = b - ((a * a) / 3.0);

        double q = (2 * Math.pow(a, 3) / 27.0) - (a * b / 3.0) + c;

        double delta = (Math.pow(q, 2) / 4) + (Math.pow(p, 3) / 27);

        if (delta > 0.001) {

            double mt1, mt2;

            double t1 = (-q / 2.0) + Math.sqrt(delta);
            double t2 = (-q / 2.0) - Math.sqrt(delta);

            if (t1 < 0) {
                mt1 = (-1) * (Math.pow(-t1, (double) 1 / 3));
            } else {
                mt1 = (Math.pow(t1, (double) 1 / 3));
            }

            if (t2 < 0) {
                mt2 = (-1) * (Math.pow(-t2, (double) 1 / 3));
            } else {
                mt2 = (Math.pow(t2, (double) 1 / 3));
            }

            x1 = mt1 + mt2 - (a / 3.0);

        } else if (delta < 0.001 && delta > -0.001) {

            if (q < 0) {

                x1 = 2 * Math.pow(-q / 2, (double) 1 / 3) - (a / 3);
                x2 = -1 * Math.pow(-q / 2, (double) 1 / 3) - (a / 3);

            } else {
                x1 = -2 * Math.pow(q / 2, (double) 1 / 3) - (a / 3);
                x2 = Math.pow(q / 2, (double) 1 / 3) - (a / 3);

            }

        } else {

            x1 = (2.0 / Math.sqrt(3)) * (Math.sqrt(-p) * Math.sin((1 / 3.0) * Math.asin(((3 * Math.sqrt(3) * q) / (2 * Math.pow(Math.pow(-p, (double) 1 / 2), 3)))))) - (a / 3.0);
            x2 = (-2.0 / Math.sqrt(3)) * (Math.sqrt(-p) * Math.sin((1 / 3.0) * Math.asin(((3 * Math.sqrt(3) * q) / (2 * Math.pow(Math.pow(-p, (double) 1 / 2), 3)))) + (Math.PI / 3))) - (a / 3.0);
            x3 = (2.0 / Math.sqrt(3)) * (Math.sqrt(-p) * Math.cos((1 / 3.0) * Math.asin(((3 * Math.sqrt(3) * q) / (2 * Math.pow(Math.pow(-p, (double) 1 / 2), 3)))) + (Math.PI / 6))) - (a / 3.0);

        }
        return x2;
    }

    public double cubicFormula(double a, double b, double c, double d) {
        double var1 = 54 * b * b * b;
        double var2 = 162 * (b * a * c - a * a * d);
        double var3 = -9 * (b * b + 2 * a * c);
        double partialSolve = sqrt((var1 + var2) * (var1 + var2) + 4 * (var3 * var3 * var3));
        return 1 / a * -.26457 * cbrt(var1 + partialSolve + var2) - (0.42 * var3) / (a * cbrt(var1 + partialSolve + var2)) + b / a;
    }

    public void calculateTList(double length) {
        tList = new ArrayList();
        double endT;
        if (c != POSITIVE_INFINITY) {
            if (startVel < velo1) {
                double t = 2 * startVel / MAX_ACCEL;
                tList.add(time - t);
                startDist = c * t * t * t / 6;
            } else if (startVel < velo2) {
                double t1 = 2 * velo1 / MAX_ACCEL;
                double t2 = (startVel - velo1) / MAX_ACCEL;
                tList.add(time - t1 - t2);
                startDist = c * t1 * t1 * t1 / 6 + velo1 * t2 + MAX_ACCEL * t2 * t2 / 2;
            } else {
                double t1 = 2 * velo1 / MAX_ACCEL;
                double t2 = (velo2 - velo1) / MAX_ACCEL;
                double t3 = 2 * (startVel - velo2) / MAX_ACCEL;
                tList.add(time - t1 - t2 - t3);
                startDist = c * t1 * t1 * t1 / 6 + velo1 * t2 + MAX_ACCEL * t2 * t2 / 2 + velo2 * t3 + MAX_ACCEL * t3 * t3 / 2
                        - c * t3 * t3 * t3 / 6;
            }
            double avgAccel = MAX_ACCEL * (1 - 0.5 * curviness);
            endVel = min(endVel, sqrt(startVel * startVel + 2 * avgAccel * length));
            if (endVel < velo1) {
                endT = 2 * (endVel) / MAX_ACCEL;
                endDist = c * endT * endT * endT / 6;
            } else if (endVel < velo2) {
                double t1 = 2 * (velo1) / MAX_ACCEL;
                double t2 = (endVel - velo1) / MAX_ACCEL;
                endT = t1 + t2;
                endDist = c * t1 * t1 * t1 / 6 + velo1 * t2 + MAX_ACCEL * t2 * t2 / 2;

            } else {
                double t1 = 2 * (velo1) / MAX_ACCEL;
                double t2 = (velo2 - velo1) / MAX_ACCEL;
                double t3 = 2 * (endVel - velo2) / MAX_ACCEL;
                endT = t1 + t2 + t3;
                endDist = c * t1 * t1 * t1 / 6 + velo1 * t2 + MAX_ACCEL * t2 * t2 / 2 + velo2 * t3 + MAX_ACCEL * t3 * t3 / 2
                        - c * t3 * t3 * t3 / 6;

            }
            totalDist = length + startDist + endDist;
            double t1 = 2 * (velo1) / MAX_ACCEL;
            double t2 = (velo2 - velo1) / MAX_ACCEL;
            double t3 = 2 * (MAX_VEL - velo2) / MAX_ACCEL;
            double phase1dist = c * t1 * t1 * t1 / 6;
            double phase2dist = velo1 * t2 + MAX_ACCEL * t2 * t2 / 2;
            double phase3dist = velo2 * t3 + MAX_ACCEL * t3 * t3 / 2 - c * t3 * t3 * t3 / 6;
            if (totalDist * 0.5 < phase1dist) {
                double t = cbrt(3 * totalDist / c);
                peakVel = t * MAX_ACCEL * 0.5;
                packet.put("peakVelCase0", t);
            } else if (totalDist * 0.5 < phase2dist + phase1dist) {
                double dist = totalDist * 0.5 - phase1dist;
                double t = -velo1 + sqrt(velo1 * velo1 + 2 * MAX_ACCEL * dist);
                t /= MAX_ACCEL;
                peakVel = velo1 + t * MAX_ACCEL;
                packet.put("peakVelCase1", peakVel);

            } else if (totalDist * 0.5 < phase3dist + phase2dist + phase1dist) {
                double dist = totalDist * 0.5 - phase1dist - phase2dist;
                double t = cubicFormula(-c / 6, MAX_ACCEL / 2, velo2, -dist);
                peakVel = velo2 + MAX_ACCEL * 0.5 * t * t;
                packet.put("peakVelCase2", peakVel);

            } else {
                peakVel = MAX_VEL;
                packet.put("peakVelCase3", peakVel);

            }

            double cruiseLength = max(length - getAccelDist(peakVel, endVel) - getAccelDist(startVel, peakVel), 0);
            tList.add(tList.get(0) + 2 * min(velo1, peakVel) / MAX_ACCEL);
            tList.add(tList.get(1) + (min(velo2, peakVel) - min(velo1, peakVel)) / MAX_ACCEL);
            tList.add(tList.get(2) + 2 * (peakVel - min(velo2, peakVel)) / MAX_ACCEL);
            tList.add(tList.get(3) + cruiseLength / peakVel);
            tList.add(tList.get(4) + 2 * (peakVel - min(velo2, peakVel) / MAX_ACCEL));
            tList.add(tList.get(5) + (min(velo2, peakVel) - min(velo1, peakVel)) / MAX_ACCEL);
            tList.add(tList.get(6) + 2 * min(velo1, peakVel) / MAX_ACCEL);
            tList.add(endT);
        } else {
            double t0 = startVel / MAX_ACCEL;
            startDist = MAX_ACCEL * t0 * t0 * 0.5;
            double avgAccel = MAX_ACCEL * (1 - 0.5 * curviness);
            endVel = min(endVel, sqrt(startVel * startVel + 2 * avgAccel * length));
            endT = endVel / MAX_ACCEL;
            endDist = MAX_ACCEL * endT * endT * 0.5;
            totalDist = length + startDist + endDist;
            peakVel = min(sqrt(totalDist * MAX_ACCEL), MAX_VEL);

            double cruiseLength = max(length - getAccelDist(peakVel, endVel) - getAccelDist(startVel, peakVel), 0);
            tList.add(time - startVel / MAX_ACCEL);
            tList.add(tList.get(0));
            tList.add(tList.get(1) + peakVel / MAX_ACCEL);
            tList.add(tList.get(2));
            tList.add(tList.get(3) + cruiseLength / peakVel);
            tList.add(tList.get(4));
            tList.add(tList.get(5) + peakVel / MAX_ACCEL);
            tList.add(tList.get(6));
            tList.add(endT);
        }
        packet.put("mpEndVel", endVel);

        packet.put("mpendDist", endDist);

        packet.put("mptotalDist", totalDist);

        packet.put("mpPeakVel", peakVel);


    }

    public double calculateTargetVelocity(double time) {
        if (time > tList.get(7) - tList.get(8)) {
            time = tList.get(7) - tList.get(8);
        }
        if (curviness != 0) {
            if (time < tList.get(1)) {
                targetVelocity = startVel + 0.5 * c * (time - tList.get(0)) * (time - tList.get(0));
                return targetVelocity;
            } else if (time < tList.get(2)) {
                targetVelocity = velo1 + MAX_ACCEL * (time - tList.get(1));
                return targetVelocity;
            } else if (time < tList.get(3)) {
                targetVelocity = peakVel - 0.5 * c * (tList.get(3) - time) * (tList.get(3) - time);
                return targetVelocity;
            } else if (time < tList.get(4)) {
                targetVelocity = peakVel;
                return targetVelocity;
            } else if (time < tList.get(5)) {
                return peakVel - 0.5 * c * (time - tList.get(4)) * (time - tList.get(4));
            } else if (time < tList.get(6)) {
                return velo2 - MAX_ACCEL * (time - tList.get(5));
            } else {
                if (time > tList.get(7)) {
                    time = tList.get(7);
                }
                return velo1 - 0.5 * c * (time - tList.get(6)) * (time - tList.get(6));
            }
        } else {
            if (time < tList.get(2)) {
                targetVelocity = MAX_ACCEL * (time - tList.get(1));
                return targetVelocity;
            } else if (time < tList.get(4)) {
                targetVelocity = peakVel;
                return targetVelocity;
            } else if (time < tList.get(6)) {
                return peakVel - MAX_ACCEL * (time - tList.get(5));
            } else {
                if (time > tList.get(7)) {
                    time = tList.get(7);
                }
                return peakVel - MAX_ACCEL * (time - tList.get(5));
            }
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
        if (curviness != 0) {
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
                        peakVel * t4);
            } else if (time > tList.get(2)) {
                double t1 = tList.get(7) - tList.get(6);
                double t2 = tList.get(6) - tList.get(5);
                double t3 = tList.get(5) - tList.get(4);
                double t4 = tList.get(4) - tList.get(3);
                double t5 = tList.get(3) - time;
                return totalDist - (c * t1 * t1 * t1 / 6 + velo1 * t2 + MAX_ACCEL * t2 * t2 * 0.5 + velo2 * t3 + MAX_ACCEL * t3 * t3 * 0.5 - c * t3 * t3 * t3 / 6 +
                        peakVel * t4 + peakVel * t5 - c * t5 * t5 * t5 / 6);
            } else if (time > tList.get(1)) {
                double t1 = tList.get(7) - tList.get(6);
                double t2 = tList.get(6) - tList.get(5);
                double t3 = tList.get(5) - tList.get(4);
                double t4 = tList.get(4) - tList.get(3);
                double t5 = tList.get(3) - tList.get(2);
                double t6 = tList.get(2) - time;
                return totalDist - (c * t1 * t1 * t1 / 6 + velo1 * t2 + MAX_ACCEL * t2 * t2 * 0.5 + velo2 * t3 + MAX_ACCEL * t3 * t3 * 0.5 - c * t3 * t3 * t3 / 6 +
                        peakVel * t4 + peakVel * t5 - c * t5 * t5 * t5 / 6 + velo2 * t6 - MAX_ACCEL * t6 * t6 * 0.5);
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
                        peakVel * t4 + peakVel * t5 - c * t5 * t5 * t5 / 6 + velo2 * t6 - MAX_ACCEL * t6 * t6 * 0.5 + velo1 * t7 - MAX_ACCEL * t7 * t7 * 0.5
                        + c * t7 * t7 * t7 / 6);
            }
        } else {
            if (time > tList.get(5)) {
                double t2 = tList.get(6) - time;
                return totalDist - MAX_ACCEL * t2 * t2 * 0.5;
            } else if (time > tList.get(3)) {
                double t2 = tList.get(6) - tList.get(5);
                double t4 = tList.get(4) - time;
                return totalDist - MAX_ACCEL * t2 * t2 * 0.5 - peakVel * t4;
            } else {
                double t2 = tList.get(6) - tList.get(5);
                double t4 = tList.get(4) - tList.get(3);
                double t6 = tList.get(2) - time;
                return totalDist - MAX_ACCEL * t2 * t2 * 0.5 - peakVel * t4 - (peakVel * t6 - MAX_ACCEL * t6 * t6 * 0.5);
            }
        }
    }

    //take in remaining dist, convert to remaining time
    public double motionProfileRemDistToRemTime(double dist) {
        dist = totalDist - dist - endDist;
        packet.put("dist", dist);
        double time = 0.5;
        for (int i = 0; i < SEARCH_RESOLUTION - 1; i++) {
            double searchDist = motionProfileTimeToDist(tList.get(7) * (time) + tList.get(0) * (1 - time));
            packet.put("time" + i, tList.get(7) * (time) + tList.get(0) * (1 - time));
            packet.put("dist" + i, searchDist);
            double error = dist - searchDist;
            if (abs(error) > 0.01) {
                if (error > 0) {
                    time += (1 * pow(2, -i - 2));
                } else {
                    time -= (1 * pow(2, -i - 2));
                }
            }
        }
        return tList.get(7) - (tList.get(7) * (time) + tList.get(0) * (1 - time));
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
