package org.firstinspires.ftc.teamcode.subsystems.swerve;

import java.util.Arrays;

public class OptimalAngleCalculator {
    public boolean[] requiresReversing = new boolean[4];


//    public OptimalAngleCalculator(double currentAngle, double vectorAngle)
//    {
//        calculateOptimalAngle(currentAngle, vectorAngle);
//    }
    // idt there are any parameters to need to be initialized (faster runtimes) but I might be wrong
    public double calculateOptimalAngle(double currentAngle, double vectorAngle, int m) {
        double[] angs = {
                vectorAngle,
                vectorAngle + 180,
                vectorAngle + 360,
                vectorAngle - 180,
                vectorAngle - 360
                };

        double[] diffs = new double[5];
        for (int i = 0; i < angs.length; i++) {
            diffs[i] = Math.abs(angs[i] - currentAngle);
        }

        int index = 0;
        double min = diffs[index];
        for (int i = 1; i < diffs.length; i++) {
            if (diffs[i] <=min) {
                min = diffs[i];
                index = i;
            }
        }

        if (index % 2 == 1) {
            requiresReversing[m] = true;
        } else {
            requiresReversing[m] = false;
        }

        return angs[index];
    }
    public boolean requiresReversing(int m) {
        return requiresReversing[m];
    }
    public boolean[] reverses(){
        return requiresReversing;
    }
    
}
