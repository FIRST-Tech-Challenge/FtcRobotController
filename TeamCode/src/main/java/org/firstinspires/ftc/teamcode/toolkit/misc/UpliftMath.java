package org.firstinspires.ftc.teamcode.toolkit.misc;

import android.util.Log;

public class UpliftMath {
    public static double truncate(double val) {
        return (((int)(val * 1000)) / 1000.0);
    }

    public static double angleRestrictions(double angle) {
        while (angle < -180) {
            angle += 360;
        }
        while (angle > 180) {
            angle -= 360;
        }
        return angle;
    }

    // inverse tangent in the Uplift angle system (returns radians)
    public static double atan2UL(double y, double x) {
        return (Math.PI / 2) - Math.atan2(y, x);
    }

    public static double slowApproach(double moveSpeed, double distToTarget, double approachZoneRadius, double tolerance) {
        if(Math.abs(distToTarget) > approachZoneRadius) {
            // NOTHING
        } else if(Math.abs(distToTarget) > tolerance) {
            double val = Math.abs(distToTarget) / (approachZoneRadius);
            moveSpeed = moveSpeed * val;
            if(tolerance > 1) {
                if (Math.abs(moveSpeed) < 0.4) {
                    moveSpeed = 0.4;
                }
            } else {
                if(Math.abs(moveSpeed) < 0.25) {
                    moveSpeed = 0.25;
                }
            }
        } else {
            moveSpeed = 0;
        }
        return Math.abs(moveSpeed);
    }

    public static double slowTurnDriving(double turnSpeed, double angleRemaining, double approachZoneDeg, double toleranceDeg) {
        if(Math.abs(angleRemaining) > Math.abs(approachZoneDeg)) {
            // NOTHING
        } else if(Math.abs(angleRemaining) > toleranceDeg) {
            double val = angleRemaining / (approachZoneDeg);
            turnSpeed = turnSpeed * val;
        } else {
            turnSpeed = 0;
        }
        Log.i("Turn Speed", turnSpeed + "");
        return Math.abs(turnSpeed);
    }

}
