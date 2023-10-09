package org.firstinspires.ftc.teamcode.src.v2.maths;

public class mathsOperations {
    //normalizes the angle given
    public static double angleWrap(double wrap){

        while(wrap <= -180) {
            wrap += 360;
        }
        while(wrap > 180) {
            wrap -= 360;
        }
        return wrap;
    }

    //replaces turning a module by 180 degrees with reversing motor power.
    public static double[] efficientTurn(double reference,double state,double power){
        double error = reference-state;

        while(error>90) {
            power *=-1;
            reference -= 180;
            error = reference-state;
        }
        while(error<-90) {
            power *=-1;
            reference += 180;
            error = reference-state;
        }

        return new double[]{reference,power};
    }

    public static boolean dynamicTurn(double error){ return Math.abs(error) > 90; }

    //converts two degrees of freedom into a differential
    public static double[] diffyConvert(double rotate, double translate){
        double m1 = rotate + translate;
        double m2 = rotate - translate;
        double maxi = Math.max(Math.abs(m1),Math.abs(m2));
        if(maxi > 1){
            m1 /= Math.abs(maxi);
            m2 /= Math.abs(maxi);
        }
        return new double[]{m1,m2};
    }

    //math for detecting when an absolute encoder has wrapped around
    public static double modWrap(double state, double wrap, double last, double ratio){
        double delta = state - last;

        if (delta > 180) wrap+=1;
        if (delta < -180) wrap +=1;
        if (wrap > ratio-1) wrap = 0;
        if (wrap == 0) return state/ratio;
        return 360/(wrap+1) + state/ratio;
    }

    public static boolean equals(double state, double equals, double thresh){
        if (Math.abs(state - equals) < thresh){
            return true;
        }
        return false;
    }

    public static double power(double x, double pow) {
        for (int i = 0; i < pow; i++){
            x *= x;
        }
        return x;
    }
}