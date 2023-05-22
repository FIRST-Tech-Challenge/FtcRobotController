package org.firstinspires.ftc.teamcode;

public class utilityFunctions {
    public static double sum(double... nums) {
        double total = 0;
        for(double num : nums) {
            total += num;
        }
        return total;
    }

    public static double average(double... nums) {
        return nums.length == 0 ? 0.0 : sum(nums) / nums.length;
    }

    public static int round(double num)
    {
        return (int) Math.floor(num + 0.5);
    }

    public static double round(double num, double precision)
    {
        return Math.round(num / precision) * precision;
    }

    public static boolean inRange(int value, int low, int high, boolean inclusive)
    {
        return inclusive ? value >= low && value <= high : value > low && value < high;
    }

    public static boolean inRange(int value, int low, int high)
    {
        return inRange(value, low, high, true);
    }
}
