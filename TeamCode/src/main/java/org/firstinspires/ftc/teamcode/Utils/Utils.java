package org.firstinspires.ftc.teamcode.Utils;

import org.ejml.simple.SimpleMatrix;
public class Utils {
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

    public static SimpleMatrix rotateBodyToGlobal(SimpleMatrix vectorBody, double th) {
        return R(th).mult(vectorBody);
    }

    public static SimpleMatrix rotateGlobalToBody(SimpleMatrix vectorGlobal, double th) {
        return R(th).invert().mult(vectorGlobal);
    }
}
