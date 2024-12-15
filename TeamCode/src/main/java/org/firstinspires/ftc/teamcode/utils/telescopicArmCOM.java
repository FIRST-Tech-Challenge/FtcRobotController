package org.firstinspires.ftc.teamcode.utils;

public class telescopicArmCOM {

    public static double calculateCenterOfMass(double[] masses, double[] lengths, double currentTotalLength) {
        int n = masses.length;
        double totalMass = 0;
        double currentLength = 0;

        for (int i = 0; i < n; i++) {
            totalMass += masses[i];
        }

        double weightedSum = 0;
        double currentLengthLastSegment = currentTotalLength - (currentLength + lengths[n - 1]);
        double f = currentLengthLastSegment / lengths[n - 1];

        for (int i = 0; i < n; i++) {
            double segmentLength = (i == n - 1) ? f * lengths[i] : lengths[i];
            double segmentCenterOfMass = currentLength + segmentLength / 2;
            weightedSum += masses[i] * segmentCenterOfMass;
            currentLength += lengths[i];
        }

        return weightedSum / totalMass;
    }


}



