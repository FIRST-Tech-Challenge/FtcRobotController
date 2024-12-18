package org.firstinspires.ftc.teamcode.Utils;

public class MedianFilter {
    private final float[] list;
    private final int windowLength;
    private final int midPoint;
    private int pointer = 0;

    public MedianFilter(int windowLength) {
        this.windowLength = windowLength;
        this.midPoint = windowLength / 2;
        list = new float[windowLength];

        for (int i = 0; i < windowLength; i++) {
            list[i] = 0;
        }
    }

    public MedianFilter(int windowLength, float initValue) {
        this.windowLength = windowLength;
        this.midPoint = windowLength / 2;
        list = new float[windowLength];

        for (int i = 0; i < windowLength; i++) {
            list[i] = initValue;
        }
    }

    public double calculate(double a) {
        //Poll and offer.
        float value = (float) a;
        list[pointer] = value;
        pointer = (pointer + 1) % windowLength;

        int maxPointer = 0;
        for (int i = 1; i < list.length; i++) {
            if (list[i] > list[maxPointer]) {
                maxPointer = i;
            }
        }

        return list[maxPointer];
    }
}
