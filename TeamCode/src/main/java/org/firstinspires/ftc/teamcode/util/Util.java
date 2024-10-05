package org.firstinspires.ftc.teamcode.util;

public class Util
{
    /**
     * Forces a number to be between a min and max by trimming it to the max or min if the number
     * is to great in magnitude.
     *
     * @param num
     * @param min
     * @param max
     * @return
     */
    public static double trim(double num, double min, double max)
    {
        if (num < min) return min;
        if (num > max) return max;
        return num;
    }
    
    public static boolean inRange(double myValue, double min, double max)
    {
        if(myValue >= min && myValue <= max)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    public static boolean inRange(int myValue, int min, int max)
    {
        if(myValue >= min && myValue <= max)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    
    public static int inBoundary(double myValue, double min, double max)
    {
        if(myValue < min )
        {
            return -1;
        }
        else if (myValue > max)
        {
            return 1;
        }
        else
            return 0;
    }

    /**
     * Returns 0 if the number is within a threshold of 0.
     *
     * @param num
     * @param deadband
     * @return
     */
    public static double applyDeadband(double num, double deadband)
    {
        if (num < deadband && num > -deadband)
            return 0;
        return num;
    }
    
    public static int arrayAverage(int[] array)
    {
        int sum = 0;
        for(int i = 0; i < array.length; i++)
            sum += array[i];
        return sum/array.length;
    }
}