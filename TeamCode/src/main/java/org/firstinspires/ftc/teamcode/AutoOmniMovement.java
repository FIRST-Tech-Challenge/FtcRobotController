package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.Arrays;

public class AutoOmniMovement {
    double angleRad; //ALWAYS SUBTRACT 45 DEG WHEN USING IN CODE, WHEEL CALC USES DIFFERENT ALIGNMENT
    double x, y, motorX, motorY, powerA, powerB;
    final double MAX_POWER = 0.75;

    double[] wheelPowers = new double[4]; //TopRight, BottomRight, TopLeft, BottomLeft

    public AutoOmniMovement (double x, double y, double angleDeg) {
        this.x = x;
        this.y = y;
        angleRad = Math.toRadians(angleDeg-45);
        Arrays.fill(wheelPowers, 0.0);
    }

    public void findPower(double currX, double currY, double wantX, double wantY){
        findPower(wantX-currX, wantY-currY);
    }

    public void findPower(double x, double y) {
        double slope = y/x;
        if (((double) Math.round(Math.sin(angleRad)*1000))/1000 == 0) {
            angleRad = Math.toRadians(Math.toDegrees(angleRad)+0.1);
        }

        motorX = Math.cos(angleRad);
        motorY = Math.sin(angleRad);

        powerA = ((motorX/1) + (motorY/slope))/2;
        powerB = ((-motorX/1) + (motorY/slope))/2;
        if (Math.abs(powerA) > Math.abs(powerB)) {
            powerB = MAX_POWER/(powerA/powerB);
            powerA = MAX_POWER;
        } else {
            powerA = MAX_POWER/(powerB/powerA);
            powerB = MAX_POWER;
        }
        if (x < 0){
            powerA = -powerA;
        } else {
            powerB = -powerB;
        }
        wheelPowers[0] = powerB;
        wheelPowers[1] = powerA;
        wheelPowers[2] = powerA;
        wheelPowers[3] = powerB;
    }

    public void addTurnPower(double angleDeg) {
        double goalAngleDeg = (angleDeg-45) - Math.toDegrees(angleRad);
        double powerDiff = 0;
        if (Math.abs(goalAngleDeg) > 0) {
            powerDiff = 1-MAX_POWER;
        }if (Math.abs(goalAngleDeg) < 90) {
            powerDiff = powerDiff/2;
        }if (Math.abs(goalAngleDeg) < 20){
            powerDiff = powerDiff/2;
        }
        powerDiff = (goalAngleDeg>0) ? powerDiff : -powerDiff;
        for (int i = 0; i <= 3; i++) {
            if (wheelPowers[i]==MAX_POWER){
                wheelPowers[i]+=powerDiff;
                powerDiff = -powerDiff;
            }
        }
    }
}
