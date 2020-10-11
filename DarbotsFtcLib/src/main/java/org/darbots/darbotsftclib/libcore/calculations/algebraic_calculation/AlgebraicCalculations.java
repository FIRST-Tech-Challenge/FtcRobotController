package org.darbots.darbotsftclib.libcore.calculations.algebraic_calculation;

public class AlgebraicCalculations {
    public static int map(int number, int originalMin, int originalMax, int newMin, int newMax){
        int oldDelta = originalMax - originalMin;
        int newDelta = newMax - newMin;
        float startDelta = (number - originalMin);
        int scaled = newMin + Math.round(startDelta / oldDelta * newDelta);
        return scaled;
    }
    public static float map(float number, float originalMin, float originalMax, float newMin, float newMax){
        return newMin + ((number-originalMin) / (originalMax - originalMin) * (newMax - newMin));
    }
    public static double map(double number, double originalMin, double originalMax, double newMin, double newMax){
        return newMin + ((number-originalMin) / (originalMax - originalMin) * (newMax - newMin));
    }
}
