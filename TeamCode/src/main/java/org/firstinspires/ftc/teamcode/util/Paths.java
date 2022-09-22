package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Curves;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.List;

public final class Paths {
    private Paths() {

    }

    // at least 6 inch resolution
    public static double project(PosePath path, Vector2d query) {
        List<Double> xs = com.acmerobotics.roadrunner.Math.range(
                0.0, path.length(), (int) Math.ceil(path.length() / 6.0));
        double champDisp = Curves.project(path, query, xs.get(0));
        final Vector2d initVec = path.get(champDisp, 1).trans.value();
        double champSqrNorm = initVec.minus(query).sqrNorm();
        for (int i = 1; i < xs.size(); i++) {
            double disp = Curves.project(path, query, xs.get(i));
            Vector2d vec = path.get(disp, 1).trans.value();
            double sqrNorm = vec.minus(query).sqrNorm();

            if (sqrNorm < champSqrNorm) {
                champDisp = disp;
                champSqrNorm = sqrNorm;
            }
        }

        return champDisp;
    }
}
