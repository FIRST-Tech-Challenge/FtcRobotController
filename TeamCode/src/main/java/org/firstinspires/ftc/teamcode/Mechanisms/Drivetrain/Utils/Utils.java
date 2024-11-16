package org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils;

import org.ejml.simple.SimpleMatrix;
public class Utils {
    public static double r = 2.16535; //in
    public static double l = 5.7; //in
    public static double w = 5.31496; //in
    public static SimpleMatrix R(double th) {
        SimpleMatrix rotation = new SimpleMatrix(
                new double[][]{
                        new double[]{Math.cos(th), -1*Math.sin(th), 0},
                        new double[]{Math.sin(th), Math.cos(th), 0},
                        new double[]{0, 0, 1},
                }
        );
        return rotation;
    }
    static SimpleMatrix H_inv = new SimpleMatrix(
            new double[][]{
                    new double[]{1d, -1d, -(l + w)},
                    new double[]{1d, 1d, -(l + w)},
                    new double[]{1d, -1d, (l + w)},
                    new double[]{1d, 1d, (l + w)}
            }
    ); //Inverse kinematics Matrix
    public static SimpleMatrix rotateBodyToGlobal(SimpleMatrix vectorBody, double th) {
        return R(th).mult(vectorBody);
    }

    public static SimpleMatrix rotateGlobalToBody(SimpleMatrix vectorGlobal, double th) {
        return R(th).invert().mult(vectorGlobal);
    }
    public static SimpleMatrix inverseKinematics(SimpleMatrix twist) {
        SimpleMatrix wheelSpeeds = H_inv.scale(1 / r).mult(twist);
        return wheelSpeeds;
    }
    public static double angleWrap(double radians) {

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        // keep in mind that the result is in radians
        return radians;
    }
    public static double calculateDistance(double x1, double y1,double x2,double y2){
        return Math.sqrt(Math.pow((x2-x1),2)+Math.pow((y2-y1),2));
    }
}
