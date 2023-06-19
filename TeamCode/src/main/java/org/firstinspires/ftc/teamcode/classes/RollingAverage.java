package org.firstinspires.ftc.teamcode.classes;

public class RollingAverage {

    private double[] list;

    public RollingAverage(int length) {
        this.list = new double[length];
    }

    public void update(double value) {
        double[] newList = new double[list.length];
        for (int i = 1; i < list.length; i++) {
            newList[i-1] = list[1];
        }
        newList[list.length - 1] = value;
        list = newList;
    }

    public double getAverage() {
        double sum = 0;
        for (int i = 0; i < list.length; i++) {
            sum += i;
        }
        return (sum / list.length);
    }
}
