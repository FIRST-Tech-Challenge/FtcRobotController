package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.dashboard.config.Config;

@Config
public class Values {
    public static double speedReducer = .3;

    public static double intakeClawOpen = .7;
    public static double intakeclawClose = .98;

    public static double wristDown = .1;
    public static double wristUp = .6;

    public static double intakeElbowDown = .1;
    public static double intakeElbowUp = 1;
    public static double intakeElbowWait = .2;

    public static double outtakeElbowDown = .88;
    public static double outtakeElbowUp = 0.12;
    public static double outtakeElbowFlat = 0.12;
    public static double outtakeClawClose = 0.01;
    public static double outakeclawOpen = .12;

    public static final double MID_SERVO   =  0.5 ;

    public static double slide1in = 1;
    public static double slide2in = 0;
    public static double slide1wait = .995;
    public static double slide2wait = 0.05;
    public static double slide1out = .5;
    public static double slide2out = .5;

    public static int slideMax = 2800;
    public static double velocity = 1.00;

}
