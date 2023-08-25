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

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

import java.util.ArrayList;

public class RFMotionProfile {
    double curviness, c, velo1, velo2, endVel, startVel, startT;
    ArrayList<Double> tList;
    double MIN_DECEL = -5, startDist, endDist, totalDist, length = 1000, targetVelocity, peakVel, distance = 0, targetAcceleration;
    int SEARCH_RESOLUTION = 12;
    double FFTime = 0.1;

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
        endVel = min(p_endVel.norm(), MAX_VEL);
        startVel = min(p_startVel.norm(), MAX_VEL);
        packet.put("mpStartVel", startVel);
    }

    public void setLength(double p_length) {
        if (length != p_length) {
            if (distance == 0) {
                distance = p_length;
            }
            length = p_length;
            calculateTList(length);
            packet.put("inpudDist1", length);
        }
    }

    public boolean isProfileDone(double time) {
        return scaledTime(time) > tList.get(7) - tList.get(8);
    }

    public double getInstantaneousTargetAcceleration(double dist,Vector2d projectedVelo, Vector2d endVelo) {
//        velo = currentVelocity.vec().norm();
//        double p_vel = velo;
        double targetAccel = 0;
        //if u should decel
        boolean shouldDecel = false;
        double xDist = getAccelDist(projectedVelo.getX(),endVelo.getX());
        double yDist = getAccelDist(projectedVelo.getY(),endVelo.getY());
        double decelDist = 1*sqrt(xDist*xDist+yDist*yDist);
        packet.put("decelDist", abs(decelDist));
        packet.put("dist", abs(dist));
        double velo = projectedVelo.norm();

        if (abs(dist) <= ((decelDist)) + 2) {
            shouldDecel = true;
            packet.put("shouldAccel", 20);

        } else {
            packet.put("shouldAccel", 0);
        }
//        boolean neg=false;
        if (velo < 0) {
//            neg=true;
//            velo *= -1;
        }
        if (velo < velo1) {
            if (shouldDecel) {
                velo = velo1 - velo;
            }
            targetAccel = c * Math.sqrt(2 * (velo) / c);
            packet.put("accelMagCase1", targetAccel);
        } else if (velo < velo2) {
            targetAccel = MAX_ACCEL;
            packet.put("accelMagCase2", targetAccel);

        } else if (velo > velo2 && velo < peakVel) {
            if (shouldDecel) {
                velo = MAX_VEL - velo;
            } else {
                velo = velo - velo2;
            }
            targetAccel = MAX_ACCEL - c * Math.sqrt(2 * (velo) / c);
            packet.put("accelMagCase3", targetAccel);

        } else {

            targetAccel = 0.0;
            packet.put("accelMagCase4", targetAccel);

        }
        if (shouldDecel) {
            targetAccel *= -1;
        }
        return targetAccel;
    }

    //v=c, c =a, a =b, d=d
    public double solveCubicEquation(double A, double B, double C, double D) {

        double x1 = 0, x2 = 0, x3 = 0;
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
        return x1;
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
                double t = sqrt(2 * startVel / c);
                tList.add(time - t);
                startDist = c * t * t * t / 6;
                startT = t;
                packet.put("strtDist1", t);

            } else if (startVel < velo2) {
                double t1 = 2 * velo1 / MAX_ACCEL;
                double t2 = (startVel - velo1) / MAX_ACCEL;
                tList.add(time - t1 - t2);
                startDist = c * t1 * t1 * t1 / 6 + velo1 * t2 + MAX_ACCEL * t2 * t2 / 2;
                startT = t1 + t2;
                packet.put("strtDist2", startT);

            } else {
                double t1 = sqrt(2 * min(velo1, startVel) / c);
                double t2 = (min(velo2, startVel) - min(startVel, velo1)) / MAX_ACCEL;
                double t3 = sqrt(2 * (startVel - min(velo2, startVel)) / c);
                tList.add(time - t1 - t2 - t3);
                startDist = c * t1 * t1 * t1 / 6 + min(velo1, startVel) * t2 + MAX_ACCEL * t2 * t2 / 2 + min(velo2, startVel) * t3 + MAX_ACCEL * t3 * t3 / 2
                        - c * t3 * t3 * t3 / 6;
                startT = t1 + t2 + t3;
                packet.put("strtDist3", t1 + t2 + t3);

            }
            //calc avgAccel
            double avgAccel;
            endVel = abs(endVel);
            if (endVel < velo1) {
                double t = sqrt(2 * endVel / c);
                avgAccel = c * t * 0.5;
                packet.put("avgAccel1", avgAccel);

            } else if (endVel < velo2) {
                double t1 = MAX_ACCEL / c;
                double t2 = (endVel - velo1) / MAX_ACCEL;
                avgAccel = MAX_ACCEL * (t2 + 0.5 * t1) / (t1 + t2);
                packet.put("avgAccel2", avgAccel);

            } else {
                endVel = min(MAX_VEL, endVel);
                double t1 = MAX_ACCEL / c;
                double t2 = (velo2 - velo1) / MAX_ACCEL;
                double t3 = sqrt(2 * (endVel - velo2) / c);
                avgAccel = MAX_ACCEL * (t2 + 0.5 * (t1 + t3)) / (t2 + t1 + t3);
                packet.put("avgAccel3", avgAccel);
            }
            packet.put("avgAccel", avgAccel);

            endVel = min(endVel, sqrt(startVel * startVel + 2 * avgAccel * length));
            if (endVel < velo1) {
                endT = sqrt(2 * (endVel) / c);
                endDist = c * endT * endT * endT / 6;
            } else if (endVel < velo2) {
                double t1 = sqrt(2 * min(velo1, endVel) / c);
                double t2 = (min(velo2, endVel) - min(endVel, velo1)) / MAX_ACCEL;
                endT = t1 + t2;
                endDist = c * t1 * t1 * t1 / 6 + velo1 * t2 + MAX_ACCEL * t2 * t2 / 2;
            } else {
                double t1 = sqrt(2 * min(velo1, endVel) / c);
                double t2 = (min(velo2, endVel) - min(endVel, velo1)) / MAX_ACCEL;
                double t3 = sqrt(2 * (endVel - min(velo2, endVel)) / c);
                endT = t1 + t2 + t3;
                endDist = c * t1 * t1 * t1 / 6 + velo1 * t2 + MAX_ACCEL * t2 * t2 / 2 + velo2 * t3 + MAX_ACCEL * t3 * t3 / 2
                        - c * t3 * t3 * t3 / 6;

            }
            packet.put("mpStartDst", startDist);
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
                packet.put("peakVelCase0", peakVel);
            } else if (totalDist * 0.5 < phase2dist + phase1dist) {
                double dist = totalDist * 0.5 - phase1dist;
                double t = -velo1 + sqrt(velo1 * velo1 + 2 * MAX_ACCEL * dist);
                t /= MAX_ACCEL;
                peakVel = velo1 + t * MAX_ACCEL;
                packet.put("peakVelCase1", peakVel);

            } else if (totalDist * 0.5 < phase3dist + phase2dist + phase1dist) {
                double dist = totalDist * 0.5 - phase1dist - phase2dist;
                double t = solveCubicEquation(-c / 6, MAX_ACCEL / 2, velo2, -dist);
                peakVel = velo2 + c * 0.5 * t * t;
                packet.put("peakVelCase2", peakVel);

            } else {
                peakVel = MAX_VEL;
                packet.put("peakVelCase3", peakVel);

            }

            double cruiseLength = max(totalDist - 2 * min((phase1dist + phase2dist + phase3dist), totalDist * 0.5), 0);
            tList.add(tList.get(0) + sqrt(2 * min(velo1, peakVel) / c));
            tList.add(tList.get(1) + (min(velo2, peakVel) - min(velo1, peakVel)) / MAX_ACCEL);
            tList.add(tList.get(2) + sqrt(2 * (peakVel - min(velo2, peakVel)) / c));
            tList.add(tList.get(3) + cruiseLength / peakVel);
            tList.add(tList.get(4) + tList.get(3) - tList.get(2));
            tList.add(tList.get(5) + tList.get(2) - tList.get(1));
            tList.add(tList.get(6) + tList.get(1) - tList.get(0));
            tList.add(endT);
        } else {
            double t0 = startVel / MAX_ACCEL;
            startDist = MAX_ACCEL * t0 * t0 * 0.5;
            double avgAccel = MAX_ACCEL;
            endVel = min(endVel, sqrt(startVel * startVel + 2 * avgAccel * length));
            endT = endVel / MAX_ACCEL;
            endDist = endVel * endT + MAX_ACCEL * endT * endT * 0.5;
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

    public double scaledTime(double time) {
        return time;
    }

    public double calculateTargetVelocity(double time) {
        time = scaledTime(time);
        if (time > tList.get(7) - tList.get(8)) {
            time = tList.get(7) - tList.get(8);
        }
        double velo2 = min(this.velo2, peakVel);
        double velo1 = min(this.velo1, peakVel);
        if (curviness != 0) {
            packet.put("mpTime", time);
            if (time < tList.get(1)) {
                packet.put("mpVCase", 10);

                targetVelocity = 0.5 * c * (time - tList.get(0)) * (time - tList.get(0));
                targetAcceleration = c*(time - tList.get(0));
                return targetVelocity;
            } else if (time < tList.get(2)) {
                packet.put("mpVCase", 20);

                targetVelocity = velo1 + MAX_ACCEL * (time - tList.get(1));
                targetAcceleration = MAX_ACCEL;
                return targetVelocity;
            } else if (time < tList.get(3)) {
                packet.put("mpVCase", 30);

                targetVelocity = peakVel - 0.5 * c * (tList.get(3) - time) * (tList.get(3) - time);
                targetAcceleration = MAX_ACCEL - c*(tList.get(3)-time);
                return targetVelocity;
            } else if (time < tList.get(4)) {
                packet.put("mpVCase", 40);

                targetVelocity = peakVel;
                targetAcceleration=0;
                return targetVelocity;
            } else if (time < tList.get(5)) {
                packet.put("mpVCase", 50);
                targetAcceleration = -c*(time-tList.get(4));
                targetVelocity = peakVel - 0.5 * c * (time - tList.get(4)) * (time - tList.get(4));
                return targetVelocity;
            } else if (time < tList.get(6)) {
                packet.put("mpVCase", 60);
                targetAcceleration = -MAX_ACCEL;
                targetVelocity = velo2 - MAX_ACCEL * (time - tList.get(5));
                return targetVelocity;
            } else {
                packet.put("mpVCase", 70);
                targetAcceleration = -c*(tList.get(7)-time);
                if (time > tList.get(7)) {
                    time = tList.get(7);
                }
                targetVelocity = 0.5 * c * (tList.get(7) - time) * (tList.get(7) - time);
                return targetVelocity;
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

    //input current time, output amount of distance traveled
    public double motionProfileTimeToDist(double time) {
//        time = scaledTime(time);
        if (time > tList.get(7) - tList.get(8)) {
            time = tList.get(7) - tList.get(8);
        }
        double velo1 = min(this.velo1, peakVel);
        double velo2 = min(this.velo2, peakVel);

        if (curviness != 0) {
            if (time > tList.get(6)) {
                double t = tList.get(7) - time;
                if (time == BasicRobot.time) {
                    packet.put("searchCase", 10);
                    packet.put("remDist", totalDist - startDist);
                    packet.put("remDist2", c * t * t * t / 6);
//                    packet.put("remDist3", totalDist - startDist - c * t * t * t / 6);
                }

                return totalDist - startDist - c * t * t * t / 6;
            } else if (time > tList.get(5)) {
                double t1 = tList.get(7) - tList.get(6);
                double t2 = tList.get(6) - time;
                if (time == BasicRobot.time) {

                    packet.put("searchCase", 20);
                    packet.put("remDist", totalDist - startDist);
                    packet.put("remDist2", c * t1 * t1 * t1 / 6 + velo1 * t2 + MAX_ACCEL * t2 * t2 * 0.5);
                    packet.put("remDist3", totalDist - startDist - (c * t1 * t1 * t1 / 6 + velo1 * t2 + MAX_ACCEL * t2 * t2 * 0.5));
                }
                return totalDist - startDist - (c * t1 * t1 * t1 / 6 + velo1 * t2 + MAX_ACCEL * t2 * t2 * 0.5);
            } else if (time > tList.get(4)) {
                double t1 = tList.get(7) - tList.get(6);
                double t2 = tList.get(6) - tList.get(5);
                double t3 = tList.get(5) - time;
                double phase1Dist = c * t1 * t1 * t1 / 6;
                double phase2Dist = velo1 * t2 + MAX_ACCEL * t2 * t2 * 0.5;
                if (time == BasicRobot.time) {
                    packet.put("searchCase", 30);
                    packet.put("remDist3", totalDist - startDist - (phase1Dist + phase2Dist + velo2 * t3 + MAX_ACCEL * t3 * t3 * 0.5 - c * t3 * t3 * t3 / 6));
                }
                return totalDist - startDist - (phase1Dist + phase2Dist + velo2 * t3 + MAX_ACCEL * t3 * t3 * 0.5 - c * t3 * t3 * t3 / 6);
            } else if (time > tList.get(3)) {
                double t1 = tList.get(7) - tList.get(6);
                double t2 = tList.get(6) - tList.get(5);
                double t3 = tList.get(5) - tList.get(4);
                double t4 = tList.get(4) - time;
                if (time == BasicRobot.time) {
                    packet.put("searchCase", 40);
                    packet.put("remDist3", totalDist - startDist - (c * t1 * t1 * t1 / 6 + velo1 * t2 + MAX_ACCEL * t2 * t2 * 0.5 + velo2 * t3 + MAX_ACCEL * t3 * t3 *
                            0.5 - c * t3 * t3 * t3 / 6 + peakVel * t4));
                }

                return totalDist - startDist - (c * t1 * t1 * t1 / 6 + velo1 * t2 + MAX_ACCEL * t2 * t2 * 0.5 + velo2 * t3 + MAX_ACCEL * t3 * t3 *
                        0.5 - c * t3 * t3 * t3 / 6 + peakVel * t4);
            } else if (time > tList.get(2)) {
                double t1 = tList.get(7) - tList.get(6);
                double t2 = tList.get(6) - tList.get(5);
                double t3 = tList.get(5) - tList.get(4);
                double t4 = tList.get(4) - tList.get(3);
                double t5 = tList.get(3) - time;
                double phase1Dist = c * t1 * t1 * t1 / 6;
                double phase2Dist = velo1 * t2 + MAX_ACCEL * t2 * t2 * 0.5;
                double phase3Dist = velo2 * t3 + MAX_ACCEL * t3 * t3 * 0.5 - c * t3 * t3 * t3 / 6;
                double phase4Dist = peakVel * t4;
                if (time == BasicRobot.time) {
                    packet.put("searchCase", 50);
                    packet.put("remDist3", totalDist - startDist - (phase1Dist + phase2Dist + phase3Dist + phase4Dist + peakVel * t5 - MAX_ACCEL * t5 * t5 * 0.5 + c
                            * t5 * t5 * t5 / 6));
                }
                return totalDist - startDist - (phase1Dist + phase2Dist + phase3Dist + phase4Dist + peakVel * t5 - MAX_ACCEL * t5 * t5 * 0.5 + c
                        * t5 * t5 * t5 / 6);
            } else if (time > tList.get(1)) {
                double t1 = tList.get(7) - tList.get(6);
                double t2 = tList.get(6) - tList.get(5);
                double t3 = tList.get(5) - tList.get(4);
                double t4 = tList.get(4) - tList.get(3);
                double t5 = tList.get(3) - tList.get(2);
                double t6 = tList.get(2) - time;
                double phase1Dist = c * t1 * t1 * t1 / 6;
                double phase2Dist = velo1 * t2 + MAX_ACCEL * t2 * t2 * 0.5;
                double phase3Dist = velo2 * t3 + MAX_ACCEL * t3 * t3 * 0.5 - c * t3 * t3 * t3 / 6;
                double phase4Dist = peakVel * t4;
                if (time == BasicRobot.time) {

                    packet.put("searchCase", 60);
                    packet.put("remDist3", totalDist - startDist - (phase1Dist + phase2Dist + 2 * phase3Dist + phase4Dist + min(velo2, peakVel) * t6 - MAX_ACCEL
                            * t6 * t6 * 0.5));
                }
                return totalDist - startDist - (phase1Dist + phase2Dist + 2 * phase3Dist + phase4Dist + min(velo2, peakVel) * t6 - MAX_ACCEL
                        * t6 * t6 * 0.5);
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
                packet.put("Mptime", time);
                double phase1Dist = c * t1 * t1 * t1 / 6;
                double phase2Dist = velo1 * t2 + MAX_ACCEL * t2 * t2 * 0.5;
                double phase3Dist = velo2 * t3 + MAX_ACCEL * t3 * t3 * 0.5 - c * t3 * t3 * t3 / 6;
                double phase4Dist = peakVel * t4;
                if (time == BasicRobot.time) {

                    packet.put("phase1Dist", phase1Dist);
                    packet.put("phase2Dist", phase2Dist);
                    packet.put("phase3Dist", phase3Dist);
                    packet.put("phase4Dist", phase4Dist);
                    packet.put("searchCase", 70);
                    packet.put("remDist3", totalDist - startDist - (phase1Dist + 2 * phase2Dist + 2 * phase3Dist + phase4Dist + velo1 * t7 - MAX_ACCEL * t7 * t7
                            * 0.5 + c * t7 * t7 * t7 / 6));
                }

                return totalDist - startDist - (phase1Dist + 2 * phase2Dist + 2 * phase3Dist + phase4Dist + velo1 * t7 - MAX_ACCEL * t7 * t7
                        * 0.5 + c * t7 * t7 * t7 / 6);
            }
        } else {
            if (time > tList.get(5)) {
                double t2 = tList.get(6) - time;
                packet.put("remDist3", totalDist - MAX_ACCEL * t2 * t2 * 0.5);

                return totalDist - MAX_ACCEL * t2 * t2 * 0.5;
            } else if (time > tList.get(3)) {
                double t2 = tList.get(6) - tList.get(5);
                double t4 = tList.get(4) - time;
                packet.put("remDist3", totalDist - MAX_ACCEL * t2 * t2 * 0.5 - peakVel * t4);

                return totalDist - MAX_ACCEL * t2 * t2 * 0.5 - peakVel * t4;
            } else {
                double t2 = tList.get(6) - tList.get(5);
                double t4 = tList.get(4) - tList.get(3);
                double t6 = tList.get(2) - time;
                packet.put("remDist3", totalDist - MAX_ACCEL * t2 * t2 * 0.5 - peakVel * t4 - (peakVel * t6 - MAX_ACCEL * t6 * t6 * 0.5));
                return totalDist - MAX_ACCEL * t2 * t2 * 0.5 - peakVel * t4 - (peakVel * t6 - MAX_ACCEL * t6 * t6 * 0.5);
            }
        }
    }

    //take in remaining dist, convert to remaining time
    public double motionProfileRemDistToRemTime(double dist) {
        dist = totalDist - dist - endDist;
        double time = 0.5;
        for (int i = 0; i < SEARCH_RESOLUTION - 1; i++) {
            double searchDist = motionProfileTimeToDist(scaledTime(tList.get(7) * (time) + tList.get(0) * (1 - time)));
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

    public double motionProfileRemainingTime(double time) {
        return tList.get(7) - tList.get(8) - scaledTime(time);
    }

    public double antiScaledTime(double time) {
        double scaledTime = time + (time - tList.get(0) - startT) * (length / distance - 1);
        return scaledTime;
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

    //WRITE A MUCH BETTTTER ONE, case when ur in final decel
    public double getAccelTime(double vel1, double vel2) {

        return abs(vel1 - vel2) / ((1 - curviness * 0.5) * MAX_ACCEL);
    }

    public double veloDecelDist(double vel) {
        double dist = 0;
        boolean neg = false;
        if (vel < 0) {
            neg = true;
            vel = abs(vel);
        }
        if (vel < velo1) {
            double t = sqrt(2 * vel / c);
            dist = c * t * t * t / 6;

        } else if (vel < velo2) {
            double t1 = 2 * velo1 / MAX_ACCEL;
            double t2 = (vel - velo1) / MAX_ACCEL;
            dist = c * t1 * t1 * t1 / 6 + velo1 * t2 + MAX_ACCEL * t2 * t2 / 2;
        } else {
            double t1 = sqrt(2 * min(velo1, vel) / c);
            double t2 = (min(velo2, vel) - min(vel, velo1)) / MAX_ACCEL;
            double t3 = sqrt(2 * (MAX_VEL - velo2) / c);
            double t4 = sqrt(2 * (MAX_VEL - vel) / c);
            dist = c * t1 * t1 * t1 / 6 + velo1 * t2 + MAX_ACCEL * t2 * t2 / 2 + velo2 * (t3 - t4)
                    + c * t3 * t3 * t3 / 6 - c * t4 * t4 * t4 / 6;
        }
        if (neg) {
            dist *= -1;
        }
        return dist;
    }

    public double getAccelDist(double vel1, double vel2) {
        if (curviness != 0) {
            return veloDecelDist(vel1) - veloDecelDist(vel2);
        } else {
            double t = (vel2 - vel1) / MAX_ACCEL;
            return vel1 * t + 0.5 * MAX_ACCEL * t * t;
        }
    }
}
