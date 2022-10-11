package org.firstinspires.ftc.teamcode.Framework.Utilities;

public class SlideConstants {

    public static int DOWN_TICKS = 15;
    public static int LOW_TICKS = 200;
    public static int MED_TICKS = 500;
    public static int HIGH_TICKS = 1000;

    public static double KP = 0.1;
    public static double KI = 0.1;
    public static double KD = 0.1;
    public static double KF = 0.1;

    public static int getTicks(SlideState state){
        switch(state){
            case DOWN:
                return DOWN_TICKS;
            case LOW:
                return LOW_TICKS;
            case MEDIUM:
                return MED_TICKS;
            case HIGH:
                return HIGH_TICKS;
        }
        return DOWN_TICKS;
    }
}
