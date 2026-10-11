package org.firstinspires.ftc.teamcode.maths;

public class mathsOperations {

    private mathsOperations() {
    }

    //normalizes the angle given
    public static double angleWrap(double wrap) {
        double angle = wrap;

        while (angle <= -180) {
            angle += 360;
        }
        while (angle > 180) {
            angle -= 360;
        }
        return angle;
    }

    //replaces turning a module by 180 degrees with reversing motor power.
    public static double[] efficientTurn(double reference, double state, double power) {
        double ref = reference;
        double pow = power;
        double error = ref - state;

        while (error > 90) {
            pow *= -1;
            ref -= 180;
            error = ref - state;
        }
        while (error < -90) {
            pow *= -1;
            ref += 180;
            error = ref - state;
        }

        return new double[]{ref, pow};
    }

    public static boolean dynamicTurn(double error) {
        return Math.abs(error) > 90;
    }

    //converts two degrees of freedom into a differential
    public static double[] diffyConvert(double rotate, double translate) {
        double m1 = rotate + translate;
        double m2 = rotate - translate;
        double maxi = Math.max(Math.abs(m1), Math.abs(m2));
        if (maxi > 1) {
            m1 /= Math.abs(maxi);
            m2 /= Math.abs(maxi);
        }
        return new double[]{m1, m2};
    }

    //math for detecting when an absolute encoder has wrapped around
    public static double modWrap(double state, double wrap, double last, double ratio) {
        double turns = wrap;
        double delta = state - last;

        if (delta > 180) turns += 1;
        if (delta < -180) turns += 1;
        if (turns > ratio - 1) turns = 0;
        if (turns == 0) return state / ratio;
        return 360 / (turns + 1) + state / ratio;
    }

    public static boolean equals(double state, double equals, double thresh) {
        return Math.abs(state - equals) < thresh;
    }

    public static double power(double x, double pow) {
        double result = x;
        int i = 0;
        while (i < pow) {
            result *= result;
            i++;
        }
        return result;
    }
}
