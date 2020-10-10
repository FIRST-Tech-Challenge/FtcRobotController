package com.technototes.library.util;

public class MathUtils {

    public static double getMax(double... args) {
        double max = args[0];
        for (int i = 1; i < args.length; i++) {
            max = Math.max(max, args[i]);
        }
        return max;
    }

    public static int getMax(int... args) {
        int max = 0;
        for (int i = 1; i < args.length; i++) {
            max = Math.max(args[i - 1], args[i]);
        }
        return max;
    }

    public static double pythag(double... vals) {
        double total = 0;
        for(double d : vals){
            total+=d*d;
        }
        return Math.sqrt(total);
    }

    public static int constrain(int min, int num, int max){
        return num < min ? min : (num > max ? max : num);

    }
    public static double constrain(double min, double num, double max){
        return num < min ? min : (num > max ? max : num);

    }
}
